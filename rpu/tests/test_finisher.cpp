// The finisher against the golden detector at the Stage 4 widths, with the PL
// blocks played by their golden models: the same tags, hamming distances,
// margins and corners, whatever order and delay the PL's answers come with.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <functional>
#include <memory>
#include <random>
#include <vector>

#include "gv/model/components.h"
#include "gv/model/decode.h"
#include "gv/model/detector.h"
#include "gv/model/refine.h"
#include "gv/rpu/finisher.h"
#include "synth.h"

using namespace gv;

namespace {

const model::Tag36h11Lookup& family() {
  static const model::Tag36h11Lookup f;
  return f;
}

model::DetectorParams golden_params() {
  model::DetectorParams p;
  p.quant = model::kStage4Widths;
  p.use_4c = false;
  return p;
}

struct RoughQuad {
  int32_t cx[4], cy[4];
};

// What the PL's quad finder emits for a frame.
std::vector<RoughQuad> rough_quads(const model::Image8& im) {
  const auto p = golden_params();
  std::vector<RoughQuad> out;
  for (const auto& c : model::label_components(model::threshold(im, p.threshold).image).closed) {
    if (c.color != 0 || c.area < p.min_component_area) continue;
    const auto r = model::scorecard_quad(c, p.quad);
    if (r.verdict != model::QuadVerdict::kOk) continue;
    RoughQuad q;
    for (size_t k = 0; k < 4; ++k) {
      q.cx[k] = static_cast<int32_t>(std::floor(r.quad.corners[k][0]));
      q.cy[k] = static_cast<int32_t>(std::floor(r.quad.corners[k][1]));
    }
    out.push_back(q);
  }
  return out;
}

// The PL's decode and edge sampler, from the golden model. Checks that a job
// only touches rows that are in.
struct FakePl {
  const model::Image8* im = nullptr;
  int rows = 1 << 20;  // rows that are in: no job may read past them
  mutable int closest = 1 << 20;  // spare rows of the tightest job that had to wait

  void touch(int row, int& last) const { last = std::max(last, std::clamp(row, 0, im->height - 1)); }
  void check_rows(int last) const {
    EXPECT_LT(last, rows) << "a job reads a row that isn't in";
    if (last < im->height - 1) closest = std::min(closest, rows - 1 - last);
  }

  void answer(rpu::Finisher& f, const rpu::ReadJob& j) const {
    model::Homography h{};
    int last = 0;
    for (size_t i = 0; i < 8; ++i) h[i] = std::ldexp(static_cast<double>(j.h[i]), i < 6 ? -14 : -24);
    h[8] = 1;
    const model::FixedProjector fp(h, model::kStage4Widths);
    ASSERT_TRUE(fp.valid);
    for (const auto& [u, v] : model::decode_sample_grid(family())) {
      const double y = std::ldexp(static_cast<double>(fp.coord(1, u, v)), -8);
      touch(static_cast<int>(std::floor(y)), last);
      touch(static_cast<int>(std::ceil(y - 0.5)), last);  // bilinear's lower row
    }
    check_rows(last);
    const auto r = model::read_code_homography(*im, h, family(), golden_params().sharpening, model::kStage4Widths);
    const auto m = r ? family().decode(r->code, golden_params().max_hamming) : std::nullopt;
    f.on_read(j.tag, r.has_value(), r ? static_cast<int32_t>(std::lround(r->decision_margin * 16)) : 0, m.has_value(),
              m ? m->id : 0, m ? m->rotation : 0, m ? m->hamming : 0);
  }
  static model::EdgeJob golden_job(const rpu::EdgeJob& j) {
    model::EdgeJob e;
    e.a = {j.a_x / 256.0, j.a_y / 256.0};
    e.d = {j.d_x / 4096.0, j.d_y / 4096.0};
    e.n = {j.n_x / 4096.0, j.n_y / 4096.0};
    e.t0 = j.t0 / 256.0, e.spacing = j.spacing / 256.0, e.count = j.count;
    e.half_steps = j.half_steps, e.window_steps = j.window_steps, e.check_direction = j.check_direction != 0;
    return e;
  }
  void answer(rpu::Finisher& f, const rpu::EdgeJob& j) const {
    const model::EdgeJob e = golden_job(j);
    int last = 0;
    for (int i = 0; i < e.count; ++i)
      for (int end : {-e.half_steps, e.half_steps}) {
        const double y = e.a[1] + e.d[1] * (e.t0 + i * e.spacing) + e.n[1] * end / 2;
        touch(static_cast<int>(std::ceil(y - 0.5)), last);
      }
    check_rows(last);
    for (const auto& p : model::sample_edge(*im, e, model::kStage4Widths))
      f.on_point(j.tag, static_cast<int32_t>(std::lround(p[0] * 256)), static_cast<int32_t>(std::lround(p[1] * 256)));
    f.on_edge_done(j.tag);
  }
};

void expect_same(const rpu::FrameResult& got, const std::vector<model::Detection>& want) {
  ASSERT_EQ(got.count, static_cast<int>(want.size()));
  for (size_t i = 0; i < want.size(); ++i) {
    const auto& g = got.tags[i];
    const auto& w = want[i];
    EXPECT_EQ(g.id, w.id);
    EXPECT_EQ(g.hamming, w.hamming);
    EXPECT_EQ(g.margin, std::lround(w.decision_margin * 16));
    EXPECT_EQ(g.cut_by_frame, w.cut_by_frame);
    for (size_t k = 0; k < 4; ++k)
      for (size_t a = 0; a < 2; ++a) EXPECT_EQ(g.corners[k][a], std::lround(w.corners[k][a] * 256)) << "tag " << g.id;
  }
}

struct Scenario {
  const char* name;
  test::SceneParams p;
  uint32_t seed;
  int scenes;
};

std::vector<Scenario> scenarios() {
  test::SceneParams full, small, blur, big;
  small.num_tags = 6, small.min_tag_px = 20, small.max_tag_px = 40;
  blur.extra_blur = 3;
  big.width = 1280, big.height = 800, big.num_tags = 4, big.min_tag_px = 150, big.max_tag_px = 500;
  return {{"full res", full, 2000, 40}, {"small tags", small, 3000, 40}, {"heavy blur", blur, 4000, 30},
          {"1280x800 large tags", big, 7000, 8}};
}

}  // namespace

TEST(Finisher, MatchesGoldenDetector) {
  std::mt19937 rng(1);
  size_t total = 0;
  for (const Scenario& s : scenarios()) {
    for (int n = 0; n < s.scenes; ++n) {
      const auto scene = test::make_scene(s.seed + static_cast<uint32_t>(n), s.p);
      const auto want = model::detect(scene.image, family(), golden_params());
      rpu::Config cfg;
      cfg.width = s.p.width, cfg.height = s.p.height;
      auto f = std::make_unique<rpu::Finisher>(cfg);
      FakePl pl{&scene.image};
      f->on_progress(5, s.p.height);
      const auto quads = rough_quads(scene.image);
      size_t sent = 0;
      bool marked = false;
      std::deque<rpu::ReadJob> reads;
      std::deque<rpu::EdgeJob> edges;
      auto out = std::make_unique<rpu::FrameResult>();
      for (int step = 0;; ++step) {
        ASSERT_LT(step, 1'000'000) << s.name << " scene " << n;
        // Any event, in a random order; each PL block answers in its own order.
        switch (rng() % 5) {
          case 0: {
            // Quads from the PL, but no more than the pool holds at once.
            const auto& k = f->counters();
            const size_t ended = k.quick_fail + k.bad_projector + k.refine_fail + k.decode_fail;
            if (sent < quads.size() && sent - ended > rpu::kMaxCandidates - 2) break;
            if (sent < quads.size()) f->on_quad(5, quads[sent].cx, quads[sent].cy), ++sent;
            else if (!marked) f->on_frame_done(5, false), marked = true;
            break;
          }
          case 1: if (rpu::ReadJob j; reads.size() < 3 && f->next_read(j)) reads.push_back(j); break;
          case 2: if (rpu::EdgeJob j; edges.size() < 3 && f->next_edge(j)) edges.push_back(j); break;
          case 3: if (!reads.empty()) pl.answer(*f, reads.front()), reads.pop_front(); break;
          default: if (!edges.empty()) pl.answer(*f, edges.front()), edges.pop_front(); break;
        }
        if (f->pop_frame(*out)) break;
      }
      EXPECT_EQ(out->frame, 5u);
      EXPECT_FALSE(out->aborted);
      EXPECT_EQ(f->counters().dropped_full + f->counters().dropped_stale, 0u);
      EXPECT_EQ(f->counters().quads, quads.size());
      SCOPED_TRACE(std::string(s.name) + " scene " + std::to_string(n));
      expect_same(*out, want);
      total += want.size();
    }
  }
  EXPECT_GT(total, 300u);
  std::cout << "[ finisher ] " << total << " tags identical to the golden detector\n";
}

// Jobs wait for their rows: with rows that aren't in filled with garbage, the
// results are still the golden ones.
TEST(Finisher, WaitsForRows) {
  test::SceneParams p;
  p.num_tags = 2, p.min_tag_px = 60, p.max_tag_px = 300;
  int tags = 0;
  for (uint32_t seed = 50; seed < 60; ++seed) {
    const auto scene = test::make_scene(seed, p);
    const auto want = model::detect(scene.image, family(), golden_params());
    rpu::Config cfg;
    cfg.width = p.width, cfg.height = p.height;
    auto f = std::make_unique<rpu::Finisher>(cfg);
    model::Image8 partial = test::noise_image(seed, p.width, p.height);
    FakePl pl{&partial};
    // Every quad up front (far earlier than the PL would send it).
    const auto quads = rough_quads(scene.image);
    ASSERT_LE(quads.size(), static_cast<size_t>(rpu::kMaxCandidates));
    for (const auto& q : quads) f->on_quad(1, q.cx, q.cy);
    f->on_frame_done(1, false);
    auto out = std::make_unique<rpu::FrameResult>();
    bool done = false;
    for (int rows = 0; rows <= p.height; ++rows) {
      f->on_progress(1, rows);
      pl.rows = rows;
      for (bool any = true; any;) {
        any = false;
        rpu::ReadJob r;
        rpu::EdgeJob e;
        while (f->next_read(r)) pl.answer(*f, r), any = true;
        while (f->next_edge(e)) pl.answer(*f, e), any = true;
      }
      done = f->pop_frame(*out) || done;
      if (rows < p.height)
        std::copy_n(scene.image.pixels.begin() + rows * p.width, p.width, partial.pixels.begin() + rows * p.width);
    }
    ASSERT_TRUE(done);
    SCOPED_TRACE(seed);
    expect_same(*out, want);
    tags += out->count;
    EXPECT_LE(pl.closest, 4);  // and jobs don't wait for much more than they need
  }
  EXPECT_GT(tags, 8);
}

// Once the next frame reaches a candidate's rows, it's dropped and counted,
// and its frame still finishes.
TEST(Finisher, DropsStaleCandidates) {
  rpu::Config cfg;
  cfg.width = 640, cfg.height = 400;
  auto f = std::make_unique<rpu::Finisher>(cfg);
  const int32_t x[4] = {100, 140, 140, 100}, top[4] = {100, 100, 140, 140}, low[4] = {300, 300, 340, 340};
  f->on_progress(0xFFFFFFFFu, 400);  // and the frame ID wraps
  f->on_quad(0xFFFFFFFFu, x, top);
  f->on_quad(0xFFFFFFFFu, x, low);
  f->on_frame_done(0xFFFFFFFFu, true);
  auto out = std::make_unique<rpu::FrameResult>();
  f->on_progress(0, 50);  // within the margin of the first quad's rows
  EXPECT_EQ(f->counters().dropped_stale, 0u);
  f->on_progress(0, 80);
  EXPECT_EQ(f->counters().dropped_stale, 1u);
  EXPECT_FALSE(f->pop_frame(*out));
  rpu::ReadJob r;
  ASSERT_TRUE(f->next_read(r));  // the lower one is still good
  f->on_progress(0, 399);        // a job in flight is never dropped
  EXPECT_EQ(f->counters().dropped_stale, 1u);
  f->on_read(r.tag, true, 100, true, 7, 0, 0);
  f->on_progress(0, 400);  // but what would follow is
  EXPECT_EQ(f->counters().dropped_stale, 2u);
  rpu::EdgeJob e;
  EXPECT_FALSE(f->next_edge(e));
  ASSERT_TRUE(f->pop_frame(*out));
  EXPECT_TRUE(out->aborted);
  EXPECT_EQ(out->count, 0);
}

// Frames finish in order, and one that overflows the pool still ends.
TEST(Finisher, OverflowAndFrameOrder) {
  rpu::Config cfg;
  auto f = std::make_unique<rpu::Finisher>(cfg);
  const int32_t x[4] = {100, 140, 140, 100}, y[4] = {100, 100, 140, 140};
  f->on_progress(1, 800);
  for (int i = 0; i < rpu::kMaxCandidates + 5; ++i) f->on_quad(1, x, y);
  EXPECT_EQ(f->counters().dropped_full, 5u);
  f->on_frame_done(1, false);
  f->on_frame_done(2, false);  // an empty frame can't overtake
  auto out = std::make_unique<rpu::FrameResult>();
  EXPECT_FALSE(f->pop_frame(*out));
  rpu::ReadJob r;
  int n = 0;
  while (f->next_read(r)) f->on_read(r.tag, false, 0, false, 0, 0, 0), ++n;
  EXPECT_EQ(n, rpu::kMaxCandidates);
  ASSERT_TRUE(f->pop_frame(*out));
  EXPECT_EQ(out->frame, 1u);
  ASSERT_TRUE(f->pop_frame(*out));
  EXPECT_EQ(out->frame, 2u);
  EXPECT_FALSE(f->pop_frame(*out));
}

// Quads no projector or line fit can handle end quietly.
TEST(Finisher, DegenerateQuads) {
  rpu::Config cfg;
  auto f = std::make_unique<rpu::Finisher>(cfg);
  f->on_progress(1, 800);
  const int32_t line_x[4] = {10, 20, 30, 40}, line_y[4] = {10, 20, 30, 40}, same[4] = {5, 5, 5, 5};
  f->on_quad(1, line_x, line_y);
  f->on_quad(1, same, same);
  f->on_frame_done(1, false);
  auto out = std::make_unique<rpu::FrameResult>();
  ASSERT_TRUE(f->pop_frame(*out));
  EXPECT_EQ(f->counters().bad_projector, 2u);
  // A quad whose edges find no points.
  const int32_t x[4] = {100, 140, 140, 100}, y[4] = {100, 100, 140, 140};
  f->on_quad(1, x, y);
  rpu::ReadJob r;
  ASSERT_TRUE(f->next_read(r));
  f->on_read(r.tag, true, 10, true, 3, 0, 0);
  rpu::EdgeJob e;
  for (int k = 0; k < 4; ++k) {
    ASSERT_TRUE(f->next_edge(e));
    f->on_edge_done(e.tag);
  }
  EXPECT_EQ(f->counters().refine_fail, 1u);
}

namespace {

// One candidate taken to its edge jobs.
std::vector<rpu::EdgeJob> edge_jobs_of(rpu::Finisher& f, const RoughQuad& q) {
  f.on_quad(1, q.cx, q.cy);
  rpu::ReadJob r;
  std::vector<rpu::EdgeJob> jobs;
  if (!f.next_read(r)) return jobs;
  f.on_read(r.tag, true, 10, true, 3, 0, 0);
  for (rpu::EdgeJob e; f.next_edge(e);) jobs.push_back(e);
  return jobs;
}

// A finished tag: quad k of a frame, with given corners (axis-aligned box) and
// margin. Its final read isn't answered.
rpu::ReadJob to_final_read(rpu::Finisher& f, int x0, int y0, int x1, int y1, uint32_t frame) {
  const int32_t cx[4] = {x0, x1, x1, x0}, cy[4] = {y0, y0, y1, y1};
  f.on_quad(frame, cx, cy);
  rpu::ReadJob r{};
  EXPECT_TRUE(f.next_read(r));
  f.on_read(r.tag, true, 10, true, 3, 0, 0);
  for (int k = 0; k < 4; ++k) {
    rpu::EdgeJob e;
    EXPECT_TRUE(f.next_edge(e));
    // Points exactly on the box's sides: horizontal and vertical lines.
    const int m = (k + 1) % 4;
    for (int i = 1; i < 8; ++i)
      f.on_point(e.tag, (cx[k] * (8 - i) + cx[m] * i) * 32, (cy[k] * (8 - i) + cy[m] * i) * 32);
    f.on_edge_done(e.tag);
  }
  EXPECT_TRUE(f.next_read(r));
  return r;
}

}  // namespace

// Every field of the edge jobs, on random quads (long, thin, tiny, skewed, and
// in either winding), against the golden refinement's jobs.
TEST(Finisher, EdgeJobsMatchGolden) {
  std::mt19937 rng(3);
  const auto image = test::noise_image(1, 640, 400);
  int jobs_checked = 0;
  for (int n = 0; n < 400; ++n) {
    std::uniform_real_distribution<double> u(0, 1);
    const double th = n % 5 ? u(rng) * 6.28 : 0, rx = 6 + u(rng) * u(rng) * 250, ry = 6 + u(rng) * u(rng) * 250;
    RoughQuad q;
    for (int k = 0; k < 4; ++k) {
      const double x = (k == 1 || k == 2 ? rx : -rx) * (0.85 + 0.3 * u(rng)), y = (k >= 2 ? ry : -ry) * (0.85 + 0.3 * u(rng));
      q.cx[k] = std::clamp(static_cast<int>(320 + x * std::cos(th) - y * std::sin(th)), 0, 639);
      q.cy[k] = std::clamp(static_cast<int>(200 + x * std::sin(th) + y * std::cos(th)), 0, 399);
    }
    if (n % 2) std::swap(q.cx[1], q.cx[3]), std::swap(q.cy[1], q.cy[3]);  // counter-clockwise
    // Golden: made clockwise, then refined.
    model::Quad rough;
    for (size_t k = 0; k < 4; ++k) rough.corners[k] = {q.cx[k] + 0.5, q.cy[k] + 0.5};
    double area2 = 0;
    for (size_t k = 0; k < 4; ++k)
      area2 += rough.corners[k][0] * rough.corners[(k + 1) % 4][1] - rough.corners[(k + 1) % 4][0] * rough.corners[k][1];
    if (area2 < 0) std::swap(rough.corners[1], rough.corners[3]);
    std::vector<model::EdgeJob> want;
    const std::function<void(const model::EdgeJob&, const std::vector<std::array<double, 2>>&)> trace =
        [&](const model::EdgeJob& j, const auto&) { want.push_back(j); };
    model::RefineParams rp;
    rp.quant = model::kStage4Widths;
    rp.trace = &trace;
    model::refine_quad(image, rough, rp);  // stops early if an edge fails
    rpu::Config cfg;
    cfg.width = 640, cfg.height = 400;
    auto f = std::make_unique<rpu::Finisher>(cfg);
    f->on_progress(1, 400);
    const auto got = edge_jobs_of(*f, q);
    if (got.empty()) continue;  // no valid projector
    ASSERT_EQ(got.size(), 4u);
    for (size_t k = 0; k < want.size(); ++k, ++jobs_checked) {
      const auto g = FakePl::golden_job(got[k]);
      EXPECT_EQ(g.a, want[k].a);
      EXPECT_EQ(g.d, want[k].d);
      EXPECT_EQ(g.n, want[k].n);
      EXPECT_EQ(g.t0, want[k].t0);
      EXPECT_EQ(g.count, want[k].count);
      EXPECT_EQ(g.half_steps, want[k].half_steps);
      EXPECT_EQ(g.window_steps, want[k].window_steps);
      ASSERT_FALSE(HasFailure()) << "quad " << n << " edge " << k;
    }
  }
  EXPECT_GT(jobs_checked, 400);
}

// Exactly horizontal and vertical edges, and the corners they meet at.
TEST(Finisher, AxisAlignedLines) {
  rpu::Config cfg;
  auto f = std::make_unique<rpu::Finisher>(cfg);
  f->on_progress(1, 800);
  const auto r = to_final_read(*f, 100, 200, 180, 264, 1);
  f->on_read(r.tag, true, 10, true, 3, 1, 0);
  f->on_frame_done(1, false);
  auto out = std::make_unique<rpu::FrameResult>();
  ASSERT_TRUE(f->pop_frame(*out));
  ASSERT_EQ(out->count, 1);
  const int want[4][2] = {{100, 264}, {100, 200}, {180, 200}, {180, 264}};  // rotation 1
  for (int k = 0; k < 4; ++k)
    for (int a = 0; a < 2; ++a) EXPECT_EQ(out->tags[0].corners[k][a], want[k][a] * 256) << k;
}

// An edge with too few points after outlier rejection fails the tag.
TEST(Finisher, MinimumPoints) {
  for (int points = 1; points <= 5; ++points) {
    rpu::Config cfg;
    auto f = std::make_unique<rpu::Finisher>(cfg);
    f->on_progress(1, 800);
    const auto jobs = edge_jobs_of(*f, RoughQuad{{100, 180, 180, 100}, {200, 200, 264, 264}});
    ASSERT_EQ(jobs.size(), 4u);
    const int32_t x[4] = {100, 180, 180, 100}, y[4] = {200, 200, 264, 264};
    for (int k = 0; k < 4; ++k) {
      for (int i = 0; i < points; ++i)
        f->on_point(jobs[static_cast<size_t>(k)].tag, (x[k] * (8 - i) + x[(k + 1) % 4] * i) * 32,
                    (y[k] * (8 - i) + y[(k + 1) % 4] * i) * 32);
      f->on_edge_done(jobs[static_cast<size_t>(k)].tag);
    }
    EXPECT_EQ(f->counters().refine_fail, points < 4 ? 1u : 0u) << points;
  }
}

// The same tag from two candidates: the better margin, in the first one's
// place, whichever finishes first. Other tags and far-apart copies stay.
TEST(Finisher, DuplicatesAndOrder) {
  for (int better_first = 0; better_first < 2; ++better_first)
    for (int reversed = 0; reversed < 2; ++reversed) {
      rpu::Config cfg;
      auto f = std::make_unique<rpu::Finisher>(cfg);
      f->on_progress(9, 800);
      const rpu::ReadJob r[4] = {
          to_final_read(*f, 100, 200, 180, 264, 9), to_final_read(*f, 300, 200, 380, 264, 9),
          to_final_read(*f, 101, 201, 181, 265, 9), to_final_read(*f, 500, 200, 580, 264, 9)};
      const int id[4] = {7, 8, 7, 7}, margin[4] = {better_first ? 90 : 50, 60, better_first ? 50 : 90, 10};
      for (int n = 0; n < 4; ++n) {
        const int i = reversed ? 3 - n : n;
        f->on_read(r[i].tag, true, margin[i], true, id[i], 0, 0);
      }
      f->on_frame_done(9, false);
      auto out = std::make_unique<rpu::FrameResult>();
      ASSERT_TRUE(f->pop_frame(*out));
      ASSERT_EQ(out->count, 3);
      EXPECT_EQ(out->tags[0].id, 7);
      EXPECT_EQ(out->tags[0].margin, 90);
      EXPECT_EQ(out->tags[0].corners[0][0], (better_first ? 100 : 101) * 256);
      EXPECT_EQ(out->tags[1].id, 8);
      EXPECT_EQ(out->tags[2].margin, 10);
      EXPECT_EQ(f->counters().duplicates, 1u);
    }
}

// A pixel store shorter than the frame: rows go when the ring wraps.
TEST(Finisher, ShortRing) {
  rpu::Config cfg;
  cfg.width = 640, cfg.height = 400, cfg.rows_kept = 128;
  auto f = std::make_unique<rpu::Finisher>(cfg);
  const int32_t x[4] = {100, 140, 140, 100}, y[4] = {100, 100, 140, 140}, low[4] = {300, 300, 340, 340};
  rpu::ReadJob r;
  f->on_progress(1, 160);
  f->on_quad(1, x, y);       // its rows, from about 95, go from row 223; the margin is 32 rows
  f->on_progress(1, 180);
  EXPECT_EQ(f->counters().dropped_stale, 0u);
  f->on_progress(1, 200);
  EXPECT_EQ(f->counters().dropped_stale, 1u);
  EXPECT_FALSE(f->next_read(r));
  f->on_quad(1, x, y);       // arrives too late altogether
  EXPECT_FALSE(f->next_read(r));
  f->on_quad(1, x, low);     // nothing in this frame overwrites rows from 295 on
  f->on_progress(1, 400);
  EXPECT_EQ(f->counters().dropped_stale, 2u);
  ASSERT_TRUE(f->next_read(r));
  f->on_read(r.tag, true, 10, true, 3, 0, 0);
  f->on_progress(2, 0);      // the next frame may
  EXPECT_EQ(f->counters().dropped_stale, 3u);
}
