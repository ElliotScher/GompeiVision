// The whole golden detector, one frame in, tags out, in the order the FPGA
// runs the blocks (PLAN "AprilTag detector"):
//   decimate -> threshold -> streaming components -> 4B quad (sanity checks)
//   -> quick decode on the rough quad -> edge refinement -> decode -> ID lookup.
// Components that fail 4B's shape check go to the 4C tracer, then the same
// path. The whole-detector comparison against
// AprilTag 3 and ground truth is in tests/test_detector.cpp and the replay tool.
#pragma once

#include <array>
#include <vector>

#include "gv/model/decode.h"
#include "gv/model/image.h"
#include "gv/model/quad.h"
#include "gv/model/refine.h"
#include "gv/model/tag_lookup.h"
#include "gv/model/threshold.h"
#include "gv/model/tracer.h"

namespace gv::model {

struct DetectorParams {
  int decimate = 1;
  ThresholdParams threshold;
  QuadParams quad;
  RefineParams refine;  // `passes` is set from `decimate` unless refine_passes > 0
  int refine_passes = 0;
  double sharpening = 0.25;       // AprilTag's decode_sharpening default
  int max_hamming = 3;            // with NAPA's touching rule for the third bit
  uint32_t min_component_area = 16;
  bool quick_decode_rough = true;  // PLAN step 5: refine only what decodes on the rough quad
  // For the quick decode, grow the rough quad outward from its center by this
  // many px of the image 4B ran on: rough corners sit 1-3 px inside the true
  // ones, which on 20-40 px tags (2.5-5 px cells) reads the wrong cells. 1 px
  // took 20-40 px tags from 230 to 235 of 240 found, at no extra refinements.
  double quick_decode_grow = 1.0;
  bool use_4c = true;
  Quantization quant;  // fixed-point emulation for refinement and decode (off by default)
  const DecodeTrace* decode_trace = nullptr;  // every code read (quick and final), for testbenches
  TracerParams tracer;
};

struct Detection {
  int id = 0;
  int hamming = 0;
  double decision_margin = 0;
  // Full-resolution image coordinates, canonical tag order: top-left,
  // top-right, bottom-right, bottom-left of the tag as printed (the CPU
  // pipeline's order).
  std::array<std::array<double, 2>, 4> corners{};
  // A corner lies outside the image: the tag is cut by the frame edge and that
  // corner is extrapolated from the visible edges, so it can be several px
  // off. Pose solvers should down-weight or skip these.
  bool cut_by_frame = false;
};

struct DetectorStats {
  int components = 0;        // black components considered (area >= min)
  int quads_ok = 0;          // passed 4B's checks
  int sent_to_4c = 0;        // failed 4B's shape check
  int quads_4c = 0;          // ...of which 4C found a quad
  int found_4c = 0;          // tags decoded from a 4C quad
  int rough_decode_fail = 0;  // rejected by the quick decode on the rough quad
  int refine_fail = 0;
  int decode_fail = 0;
  int duplicates = 0;         // same tag found from more than one component
};

std::vector<Detection> detect(const Image8& image, const Tag36h11Lookup& family, const DetectorParams& params = {},
                              DetectorStats* stats = nullptr);

// The path after quad finding, for quads from elsewhere (e.g. AprilTag's own
// 4A quads, to compare quad finders). `rough` is in the coordinates of the
// image decimated by params.decimate, in either winding order.
std::vector<Detection> detect_from_rough_quads(const Image8& image, const Tag36h11Lookup& family,
                                               const std::vector<Quad>& rough, const DetectorParams& params = {},
                                               DetectorStats* stats = nullptr);

}  // namespace gv::model
