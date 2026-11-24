#include "gv/rpu/finisher.h"

#include <cmath>

namespace gv::rpu {

namespace {

constexpr int kScaleFrac = 14, kPerspFrac = 24, kOutFrac = 8;  // the PL projector's formats
constexpr int kSpacing = 384;                                  // 1.5 px in Q.8

double length(double x, double y) { return std::sqrt(x * x + y * y); }

int32_t to_fixed(double v, double scale) { return static_cast<int32_t>(std::round(v * scale)); }

int32_t clamp_row(int32_t v, int32_t hi) { return v < 0 ? 0 : (v > hi ? hi : v); }

}  // namespace

Finisher::Finisher(const Config& config) : config_(config) {
  for (Candidate& c : candidates_) c.state = State::kFree;
  for (Frame& f : frames_) f.used = false;
}

uint32_t Finisher::tag_of(const Candidate& c, int which) const {
  return static_cast<uint32_t>(&c - candidates_) * 4 + static_cast<uint32_t>(which);
}

int Finisher::frame_slot(uint32_t frame, bool create) {
  int free_slot = -1;
  for (int i = 0; i < kMaxFrames; ++i) {
    if (frames_[i].used && frames_[i].id == frame && !frames_[i].marked) return i;
    if (!frames_[i].used && free_slot < 0) free_slot = i;
  }
  if (!create || free_slot < 0) return -1;
  Frame& f = frames_[free_slot];
  f.used = true, f.marked = false, f.aborted = false;
  f.id = frame, f.order = next_order_++;
  f.outstanding = 0, f.count = 0;
  return free_slot;
}

void Finisher::on_quad(uint32_t frame, const int32_t cx[4], const int32_t cy[4]) {
  ++counters_.quads;
  const uint32_t seq = next_seq_++;
  const int slot = frame_slot(frame, true);
  Candidate* c = nullptr;
  for (Candidate& k : candidates_)
    if (k.state == State::kFree) {
      c = &k;
      break;
    }
  if (slot < 0 || !c) {
    ++counters_.dropped_full;
    return;
  }
  // Decode and refinement expect clockwise-on-screen corners.
  int64_t area2 = 0;
  for (int k = 0; k < 4; ++k) {
    const int m = (k + 1) % 4;
    area2 += static_cast<int64_t>(cx[k]) * cy[m] - static_cast<int64_t>(cx[m]) * cy[k];
  }
  for (int k = 0; k < 4; ++k) {
    const int from = area2 < 0 ? (4 - k) % 4 : k;
    c->rough[k][0] = cx[from] + 0.5;
    c->rough[k][1] = cy[from] + 0.5;
  }
  c->seq = seq;
  c->frame_slot = static_cast<uint8_t>(slot);
  ++frames_[slot].outstanding;
  // The quick read's quad: rough corners sit 1-3 px inside the true ones.
  double mx = 0, my = 0, grown[4][2];
  for (const auto& r : c->rough) mx += r[0] / 4, my += r[1] / 4;
  for (int k = 0; k < 4; ++k) {
    const double dx = c->rough[k][0] - mx, dy = c->rough[k][1] - my, len = length(dx, dy);
    grown[k][0] = c->rough[k][0] + dx / len * config_.quick_grow;
    grown[k][1] = c->rough[k][1] + dy / len * config_.quick_grow;
  }
  c->state = State::kQuickReady;
  if (!setup_read(*c, grown)) {
    ++counters_.bad_projector;
    finish(*c);
  }
}

// Heckbert's square-to-quad mapping, in the projector's fixed point. False if
// the projector can't take it: coefficients too big, or a sample point behind
// the camera or beyond +-32,768 px (checked at the 4 extreme samples, which
// bound the rest).
bool Finisher::setup_read(Candidate& c, const double q[4][2]) {
  const double x0 = q[0][0], y0 = q[0][1], x1 = q[1][0], y1 = q[1][1];
  const double x2 = q[2][0], y2 = q[2][1], x3 = q[3][0], y3 = q[3][1];
  const double sx = x0 - x1 + x2 - x3, sy = y0 - y1 + y2 - y3;
  const double dx1 = x1 - x2, dx2 = x3 - x2, dy1 = y1 - y2, dy2 = y3 - y2;
  const double den = dx1 * dy2 - dx2 * dy1;
  const double g = (sx * dy2 - dx2 * sy) / den;
  const double h = (dx1 * sy - sx * dy1) / den;
  const double m[8] = {x1 - x0 + g * x1, x3 - x0 + h * x3, x0, y1 - y0 + g * y1, y3 - y0 + h * y3, y0, g, h};
  for (int i = 0; i < 8; ++i) {
    const double s = std::round(m[i] * (i < 6 ? 1 << kScaleFrac : 1 << kPerspFrac));
    if (!(std::fabs(s) < 2147483648.0)) return false;  // also NaN
    c.read.h[i] = static_cast<int32_t>(s);
  }
  int64_t row_lo = INT64_MAX, row_hi = INT64_MIN;
  for (int corner = 0; corner < 4; ++corner) {
    const int64_t u = corner & 1 ? 17 : -1, v = corner & 2 ? 17 : -1;
    const int64_t d = c.read.h[6] * u + c.read.h[7] * v + (int64_t{16} << kPerspFrac);
    if (d <= 0) return false;
    for (int axis = 0; axis < 2; ++axis) {
      const int32_t* a = c.read.h + 3 * axis;
      const int64_t n = (a[0] * u + a[1] * v + int64_t{16} * a[2]) * (int64_t{1} << (kPerspFrac + kOutFrac - kScaleFrac));
      // round(|n| / d) < 2^23, without dividing.
      const int64_t abs_n = n < 0 ? -n : n;
      if (abs_n > (int64_t{1} << 60) || (d >> 39) != 0 || 2 * abs_n + d >= d * (int64_t{1} << 24)) return false;
      if (axis == 1) {
        const int64_t y = n / d / 256;  // rows, to within one
        if (y < row_lo) row_lo = y;
        if (y > row_hi) row_hi = y;
      }
    }
  }
  c.row_min = clamp_row(static_cast<int32_t>(row_lo) - 2, config_.height - 1);
  c.row_max = clamp_row(static_cast<int32_t>(row_hi) + 2, config_.height - 1);
  return true;
}

void Finisher::setup_edges(Candidate& c) {
  double mx = 0, my = 0, sides = 0;
  for (int k = 0; k < 4; ++k) {
    const int m = (k + 1) % 4;
    mx += c.rough[k][0] / 4, my += c.rough[k][1] / 4;
    sides += length(c.rough[m][0] - c.rough[k][0], c.rough[m][1] - c.rough[k][1]);
  }
  const double cell = sides / 4 / 8;  // tag36h11's black border is 8 cells across
  double window = 0.5 * cell + 1;
  window = window < config_.window_min ? config_.window_min : (window > config_.window_max ? config_.window_max : window);
  double lo = c.rough[0][1], hi = lo;
  for (int k = 0; k < 4; ++k) {
    const double* a = c.rough[k];
    const double* b = c.rough[(k + 1) % 4];
    const double len = length(b[0] - a[0], b[1] - a[1]);
    EdgeJob& j = c.edges[k];
    j.tag = tag_of(c, k);
    j.a_x = to_fixed(a[0], 256), j.a_y = to_fixed(a[1], 256);
    j.d_x = to_fixed((b[0] - a[0]) / len, 4096), j.d_y = to_fixed((b[1] - a[1]) / len, 4096);
    j.n_x = j.d_y, j.n_y = -j.d_x;  // a normal; flip it to point away from the center
    if (((a[0] + b[0]) / 2 - mx) * j.n_x + ((a[1] + b[1]) / 2 - my) * j.n_y < 0) j.n_x = -j.n_x, j.n_y = -j.n_y;
    const double skip = 0.5 * cell < len / 4 ? 0.5 * cell : len / 4;
    j.t0 = to_fixed(skip, 256);
    j.spacing = kSpacing;
    // Samples at t0 + i * 1.5 up to len - skip.
    const double first = j.t0 / 256.0, last = len - skip + 1e-9;
    j.count = last < first ? 0 : static_cast<int32_t>((last - first) / 1.5);
    while (first + j.count * 1.5 <= last) ++j.count;
    while (j.count > 0 && first + (j.count - 1) * 1.5 > last) --j.count;
    j.half_steps = static_cast<int32_t>(std::ceil(window / 0.5)) + 1;
    j.window_steps = static_cast<int32_t>(std::floor(2 * window + 1e-9));
    j.check_direction = 1;
    if (a[1] < lo) lo = a[1];
    if (a[1] > hi) hi = a[1];
  }
  const double reach = window + 2;  // half_steps is up to a pixel more, and bilinear reads one more row
  c.row_min = clamp_row(static_cast<int32_t>(lo - reach) - 1, config_.height - 1);
  c.row_max = clamp_row(static_cast<int32_t>(hi + reach) + 1, config_.height - 1);
  c.edges_done = 0;
  c.state = State::kEdgesReady;
}

// Total least squares through the first n points. The sums are exact integers;
// the direction comes from half-angle formulas, without trigonometry.
bool Finisher::fit(Line& line, int n) const {
  line.valid = false;
  if (n < 2) return false;
  int64_t sx = 0, sy = 0, sxx = 0, sxy = 0, syy = 0;
  for (int i = 0; i < n; ++i) {
    const int64_t x = points_[i][0], y = points_[i][1];
    sx += x, sy += y, sxx += x * x, sxy += x * y, syy += y * y;
  }
  const double cxx = static_cast<double>(n * sxx - sx * sx), cyy = static_cast<double>(n * syy - sy * sy);
  const double cxy = static_cast<double>(n * sxy - sx * sy);
  line.px = static_cast<double>(sx) / n / 256, line.py = static_cast<double>(sy) / n / 256;
  // The direction's angle is half that of (cxx - cyy, 2 cxy).
  const double p = cxx - cyy, q = 2 * cxy, r = length(p, q);
  if (r == 0) {
    line.dx = 1, line.dy = 0;  // as atan2(0, 0) / 2 = 0
  } else if (p >= 0) {
    line.dx = std::sqrt((1 + p / r) / 2);
    line.dy = q / r / (2 * line.dx);
  } else {
    line.dy = std::sqrt((1 - p / r) / 2);
    line.dx = q / r / (2 * line.dy);
  }
  line.valid = true;
  return true;
}

Finisher::Rows Finisher::rows_of(const Candidate& c) const {
  if (progress_rows_ < 0) return Rows::kWait;
  const uint32_t frame = frames_[c.frame_slot].id;
  if (frame == progress_frame_) {
    // Row r's slot is written again by row r + rows_kept.
    const int32_t overwritten_by = c.row_min + config_.rows_kept;
    if (overwritten_by < config_.height && progress_rows_ + config_.stale_margin_rows >= overwritten_by)
      return Rows::kGone;
    return progress_rows_ > c.row_max ? Rows::kReady : Rows::kWait;
  }
  // An older frame: its rows are there until the new frame reaches them.
  if (static_cast<int32_t>(progress_frame_ - frame) > 0) {
    // (With a ring shorter than a frame, any slot may be the next to go.)
    const bool next = progress_frame_ - frame == 1 && config_.rows_kept >= config_.height;
    return next && progress_rows_ + config_.stale_margin_rows < c.row_min ? Rows::kReady : Rows::kGone;
  }
  return Rows::kWait;  // its progress hasn't arrived yet
}

void Finisher::free_stale() {
  for (Candidate& c : candidates_) {
    if (c.state != State::kQuickReady && c.state != State::kEdgesReady && c.state != State::kFinalReady) continue;
    if (sending_ >= 0 && &c == &candidates_[sending_]) continue;
    if (rows_of(c) == Rows::kGone) {
      ++counters_.dropped_stale;
      finish(c);
    }
  }
}

void Finisher::on_progress(uint32_t frame, int32_t rows) {
  progress_frame_ = frame, progress_rows_ = rows;
  free_stale();
}

bool Finisher::next_read(ReadJob& job) {
  Candidate* best = nullptr;
  for (Candidate& c : candidates_) {
    if (c.state != State::kQuickReady && c.state != State::kFinalReady) continue;
    if (best && static_cast<int32_t>(c.seq - best->seq) > 0) continue;
    if (rows_of(c) == Rows::kReady) best = &c;
  }
  if (!best) return false;
  job = best->read;
  job.tag = tag_of(*best, 0);
  best->state = best->state == State::kQuickReady ? State::kQuickBusy : State::kFinalBusy;
  return true;
}

bool Finisher::next_edge(EdgeJob& job) {
  if (sending_ < 0) {
    for (int i = 0; i < kMaxCandidates; ++i) {
      const Candidate& c = candidates_[i];
      if (c.state != State::kEdgesReady) continue;
      if (sending_ >= 0 && static_cast<int32_t>(c.seq - candidates_[sending_].seq) > 0) continue;
      if (rows_of(c) == Rows::kReady) sending_ = i;
    }
    if (sending_ < 0) return false;
    send_edge_ = 0;
    candidates_[sending_].state = State::kEdgesBusy;
  }
  job = candidates_[sending_].edges[send_edge_];
  if (++send_edge_ == 4) sending_ = -1;
  return true;
}

void Finisher::on_read(uint32_t tag, bool ok, int32_t margin, bool found, int32_t id, int32_t rotation,
                       int32_t hamming) {
  Candidate& c = candidates_[(tag / 4) % kMaxCandidates];
  if (c.state == State::kQuickBusy) {
    if (ok && found) {
      setup_edges(c);
    } else {
      ++counters_.quick_fail;
      finish(c);
    }
    return;
  }
  if (c.state != State::kFinalBusy) return;
  if (!ok || !found) {
    ++counters_.decode_fail;
    finish(c);
    return;
  }
  Frame& f = frames_[c.frame_slot];
  if (f.count == kMaxDetections) {
    ++counters_.dropped_detections;
    finish(c);
    return;
  }
  Detection& d = f.tags[f.count++];
  d.id = id, d.hamming = hamming, d.margin = margin, d.seq = c.seq, d.cut_by_frame = false;
  // The read is the canonical code turned `rotation` times counter-clockwise,
  // so canonical corner k sits at quad corner (k - rotation) mod 4.
  for (int k = 0; k < 4; ++k) {
    const int32_t* q = c.refined[(k - rotation + 4) % 4];
    d.corners[k][0] = q[0], d.corners[k][1] = q[1];
    d.cut_by_frame |= q[0] < 0 || q[1] < 0 || q[0] > config_.width * 256 || q[1] > config_.height * 256;
  }
  finish(c);
}

void Finisher::on_point(uint32_t, int32_t x, int32_t y) {
  if (num_points_ == kMaxPoints) {
    ++counters_.points_lost;
    return;
  }
  points_[num_points_][0] = x, points_[num_points_][1] = y;
  ++num_points_;
}

void Finisher::on_edge_done(uint32_t tag) {
  Candidate& c = candidates_[(tag / 4) % kMaxCandidates];
  const int n = num_points_;
  num_points_ = 0;
  if (c.state != State::kEdgesBusy) return;
  Line& line = c.lines[tag % 4];
  if (fit(line, n)) {
    // Drop outliers, so a short obstruction doesn't drag a corner, and refit.
    int kept = 0;
    for (int i = 0; i < n; ++i) {
      const double x = points_[i][0] / 256.0 - line.px, y = points_[i][1] / 256.0 - line.py;
      if (std::fabs(x * line.dy - y * line.dx) > config_.outlier_px) continue;
      points_[kept][0] = points_[i][0], points_[kept][1] = points_[i][1];
      ++kept;
    }
    if (kept < config_.min_points) line.valid = false;
    else fit(line, kept);
  }
  if (++c.edges_done < 4) return;

  // Corner k is where edge k-1 (into it) meets edge k (out of it).
  bool ok = true;
  double corners[4][2];
  for (int k = 0; k < 4 && ok; ++k) {
    const Line& a = c.lines[(k + 3) % 4];
    const Line& b = c.lines[k];
    const double den = a.dx * b.dy - a.dy * b.dx;
    ok = a.valid && b.valid && std::fabs(den) >= 1e-9;
    if (!ok) break;
    const double t = ((b.px - a.px) * b.dy - (b.py - a.py) * b.dx) / den;
    const double x = std::round((a.px + t * a.dx) * 256), y = std::round((a.py + t * a.dy) * 256);
    ok = std::fabs(x) < 1e9 && std::fabs(y) < 1e9;
    if (!ok) break;
    c.refined[k][0] = static_cast<int32_t>(x), c.refined[k][1] = static_cast<int32_t>(y);
    corners[k][0] = x / 256, corners[k][1] = y / 256;
  }
  if (!ok) {
    ++counters_.refine_fail;
    finish(c);
    return;
  }
  c.state = State::kFinalReady;
  if (!setup_read(c, corners)) {
    ++counters_.bad_projector;
    finish(c);
  }
}

void Finisher::finish(Candidate& c) {
  --frames_[c.frame_slot].outstanding;
  c.state = State::kFree;
}

void Finisher::on_frame_done(uint32_t frame, bool aborted) {
  const int slot = frame_slot(frame, true);
  if (slot < 0) {
    ++counters_.dropped_full;
    return;
  }
  frames_[slot].marked = true;
  frames_[slot].aborted = aborted;
}

bool Finisher::pop_frame(FrameResult& out) {
  Frame* f = nullptr;
  for (Frame& k : frames_)
    if (k.used && (!f || static_cast<int32_t>(k.order - f->order) < 0)) f = &k;
  if (!f || !f->marked || f->outstanding != 0) return false;
  out.frame = f->id, out.aborted = f->aborted, out.count = 0;
  // In the quads' order, whatever order they finished in; the same tag from
  // two candidates keeps the better margin, in the first one's place.
  for (int i = 1; i < f->count; ++i) {
    const Detection d = f->tags[i];
    int j = i;
    for (; j > 0 && static_cast<int32_t>(f->tags[j - 1].seq - d.seq) > 0; --j) f->tags[j] = f->tags[j - 1];
    f->tags[j] = d;
  }
  for (int i = 0; i < f->count; ++i) {
    const Detection& d = f->tags[i];
    bool duplicate = false;
    for (int j = 0; j < out.count && !duplicate; ++j) {
      Detection& e = out.tags[j];
      if (e.id != d.id) continue;
      const double dx = (e.corners[0][0] - d.corners[0][0]) / 256.0, dy = (e.corners[0][1] - d.corners[0][1]) / 256.0;
      if (length(dx, dy) >= 4.0) continue;
      duplicate = true;
      ++counters_.duplicates;
      if (d.margin > e.margin) e = d;
    }
    if (!duplicate) out.tags[out.count++] = d;
  }
  f->used = false;
  return true;
}

}  // namespace gv::rpu
