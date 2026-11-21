#include "gv/model/detector.h"

#include <cmath>

#include "gv/model/components.h"
#include "gv/model/decimate.h"
#include "gv/model/decode.h"

namespace gv::model {

namespace {

std::optional<TagMatch> decode_quad(const Image8& im, const std::array<std::array<double, 2>, 4>& q,
                                    const Tag36h11Lookup& family, const DetectorParams& p, double* margin) {
  const auto read = read_code(im, q, family, p.sharpening);
  if (!read) return std::nullopt;
  if (margin) *margin = read->decision_margin;
  return family.decode(read->code, p.max_hamming);
}

}  // namespace

namespace {

// The path after quad finding, for one rough quad (decimated coordinates).
class Finisher {
 public:
  Finisher(const Image8& image, const Tag36h11Lookup& family, const DetectorParams& p, DetectorStats& st)
      : image_(image), family_(family), p_(p), st_(st), rp_(p.refine) {
    rp_.passes = p.refine_passes > 0 ? p.refine_passes : (p.decimate > 1 ? 2 : 1);
    if (p.decimate > 1 && rp_.first_pass_window_max <= 0) rp_.first_pass_window_max = rp_.window_max * p.decimate;
  }

  void operator()(Quad small_quad, bool from_4c) {
    // Decode and refinement expect clockwise-on-screen corners.
    double area2 = 0;
    for (size_t k = 0; k < 4; ++k)
      area2 += small_quad.corners[k][0] * small_quad.corners[(k + 1) % 4][1] -
               small_quad.corners[(k + 1) % 4][0] * small_quad.corners[k][1];
    if (area2 < 0) std::swap(small_quad.corners[1], small_quad.corners[3]);
    const Quad rough = to_full_resolution(small_quad, p_.decimate);
    auto grown = rough.corners;
    if (p_.quick_decode_grow > 0) {
      double cx = 0, cy = 0;
      for (const auto& c : grown) cx += c[0] / 4, cy += c[1] / 4;
      for (auto& c : grown) {
        const double dx = c[0] - cx, dy = c[1] - cy, len = std::hypot(dx, dy);
        const double g = p_.quick_decode_grow * p_.decimate;
        c = {c[0] + dx / len * g, c[1] + dy / len * g};
      }
    }
    if (p_.quick_decode_rough && !decode_quad(image_, grown, family_, p_, nullptr)) {
      ++st_.rough_decode_fail;
      return;
    }
    const auto refined = refine_quad(image_, rough, rp_);
    if (!refined) {
      ++st_.refine_fail;
      return;
    }
    double margin = 0;
    const auto match = decode_quad(image_, refined->corners, family_, p_, &margin);
    if (!match) {
      ++st_.decode_fail;
      return;
    }
    Detection d;
    d.id = match->id;
    d.hamming = match->hamming;
    d.decision_margin = margin;
    // The read is the canonical code turned `rotation` times counter-clockwise,
    // so the canonical corner k sits at quad corner (k - rotation) mod 4
    // (the quad starts at tag coordinate (-1, -1) and runs clockwise on screen).
    for (int k = 0; k < 4; ++k)
      d.corners[static_cast<size_t>(k)] = refined->corners[static_cast<size_t>((k - match->rotation + 4) % 4)];
    for (const auto& c : d.corners)
      d.cut_by_frame |= c[0] < 0 || c[1] < 0 || c[0] > image_.width || c[1] > image_.height;
    // The same tag from two candidates: keep the better margin.
    for (Detection& e : out) {
      if (e.id != d.id) continue;
      if (std::hypot(e.corners[0][0] - d.corners[0][0], e.corners[0][1] - d.corners[0][1]) < 4.0) {
        ++st_.duplicates;
        if (d.decision_margin > e.decision_margin) e = d;
        return;
      }
    }
    st_.found_4c += from_4c;
    out.push_back(d);
  }

  std::vector<Detection> out;

 private:
  const Image8& image_;
  const Tag36h11Lookup& family_;
  const DetectorParams& p_;
  DetectorStats& st_;
  RefineParams rp_;
};

}  // namespace

std::vector<Detection> detect(const Image8& image, const Tag36h11Lookup& family, const DetectorParams& p,
                              DetectorStats* stats) {
  DetectorStats local;
  DetectorStats& st = stats ? *stats : local;
  st = {};
  const Image8 small = decimate(image, p.decimate);
  const Labeling l = label_components(threshold(small, p.threshold).image);
  Finisher finish(image, family, p, st);
  for (const Component& c : l.closed) {
    if (c.color != 0 || c.area < p.min_component_area) continue;
    ++st.components;
    const QuadResult q = scorecard_quad(c, p.quad);
    if (q.verdict == QuadVerdict::kOk) {
      ++st.quads_ok;
      finish(q.quad, false);
    } else if (q.verdict == QuadVerdict::kExtremesOffQuad) {
      // Shape failures go to 4C; dots and tiny or solid blobs can't be tags.
      ++st.sent_to_4c;
      if (!p.use_4c) continue;
      if (const auto t = trace_quad(l, c, p.tracer)) {
        ++st.quads_4c;
        finish(*t, true);
      }
    }
  }
  return std::move(finish.out);
}

std::vector<Detection> detect_from_rough_quads(const Image8& image, const Tag36h11Lookup& family,
                                               const std::vector<Quad>& rough, const DetectorParams& p,
                                               DetectorStats* stats) {
  DetectorStats local;
  DetectorStats& st = stats ? *stats : local;
  st = {};
  Finisher finish(image, family, p, st);
  for (const Quad& q : rough) {
    ++st.quads_ok;
    finish(q, false);
  }
  return std::move(finish.out);
}

}  // namespace gv::model
