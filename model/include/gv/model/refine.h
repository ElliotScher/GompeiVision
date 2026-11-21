// Edge refinement golden model (PLAN step 5): rough quad in, subpixel corners
// out, on the full-resolution grayscale image.
//
// For each edge, every `sample_spacing` px (skipping half a cell at each end,
// where corners are rounded), sample the image across the edge along its
// outward normal and find the edge in a +/-W px window:
//   - Edge position: centroid of the strongest gradient lobe (NAPA's
//     lobe_centroid), with bilinear sampling every 0.5 px.
//   - Direction check: only an edge that goes dark-to-bright moving outward
//     counts, which is the tag's outer border edge. The neighbors on either side
//     go the other way: one cell in is the border's inner edge (bright-to-dark
//     outward), and one cell out is the quiet zone's outer edge. So the nearest
//     wrong edge the search could accept is about 2 cells away.
//   - Window: W = clamp(0.5 * cell + 1, 2, 6) px, with cell = mean side / 8.
//     Small tags get small windows, where wrong edges are close. Big tags get
//     big ones, where rough corners are furthest off. The cap bounds the
//     worst-case latency. (Decided 2026-11-20; the alternatives stay selectable
//     for the Stage 1 comparison.)
// Measured on synthetic scenes (test_refine): median corner error 0.03-0.15 px,
// and no corner over 1 px in the full-resolution, small-tag and blur cases.
// Samples whose search line leaves the image are dropped, so a tag cut by
// the frame edge is fit from its visible edges.
// Then fit a line to each edge's points with outlier rejection, so a short
// obstruction doesn't drag a corner, and intersect neighboring lines.
//
// Coordinates follow the synthetic scenes and the threshold model: pixel (x, y)
// covers [x, x+1) x [y, y+1), so its center is (x + 0.5, y + 0.5). This model
// uses doubles; the RTL's fixed-point widths come from Stage 4's error budget.
#pragma once

#include <array>
#include <optional>

#include "gv/model/image.h"
#include "gv/model/quad.h"

namespace gv::model {

enum class WindowPolicy { kScaled, kFixed };

struct RefineParams {
  WindowPolicy window = WindowPolicy::kScaled;
  double fixed_window = 5.0;   // px, used with kFixed
  double window_min = 2.0, window_max = 6.0;
  bool check_direction = true;
  double sample_spacing = 1.5;  // px along the edge
  double outlier_px = 1.0;      // drop edge points farther than this from the first fit
  int min_points = 4;           // per edge, after outlier rejection
  // Later passes start from the previous pass's corners. One pass is enough
  // when 4B ran at full resolution (the FPGA plan); use two when it ran on a
  // decimated image, whose rough corners can be beyond the window cap.
  int passes = 1;
  // Window cap for the first pass only (0: window_max). Rough quads from an
  // image decimated by f can be about f times further off, so the detector sets
  // this to window_max * f; later passes start close and use window_max.
  double first_pass_window_max = 0;
};

struct RefinedQuad {
  std::array<std::array<double, 2>, 4> corners{};  // same order as the rough quad
  std::array<int, 4> points_used{};                // per edge, after outlier rejection
  double window = 0;                               // px, as applied
};

// `rough` is in full-resolution pixel coordinates (see to_full_resolution).
std::optional<RefinedQuad> refine_quad(const Image8& image, const Quad& rough, const RefineParams& params = {});

// Maps a quad found on an image decimated by `factor` to full resolution:
// decimated pixel sx samples full pixel sx * factor, so a decimated
// coordinate u becomes (u - 0.5) * factor + 0.5.
Quad to_full_resolution(const Quad& q, int factor);

}  // namespace gv::model
