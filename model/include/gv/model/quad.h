// Quad finding 4B, the corner scorecard (PLAN step 4B). A rough quad straight
// from a closed component's 16 extreme points, with no edge points stored or
// sorted. Edge refinement (step 5) turns the rough corners into subpixel ones.
//
// Each corner of a convex quad is the extreme point for a range of directions
// equal to 180 degrees minus its interior angle, so 16 directions catch every
// corner up to about 157 degrees. The steps:
//   1. Take the extremes in direction order (they're the blob's convex-hull
//      vertices in order) and merge neighbors closer than merge_distance.
//   2. Keep the 4 that enclose the largest area.
//   3. Sanity checks, which send a component to the 4C contour tracer instead.
//      A component fails if it has fewer than 4 distinct extremes, if any other
//      extreme lies more than max(max_outside px, max_outside_frac * mean
//      side) outside the quad (rounded corners scale with nothing, merges scale
//      with the blob; the blob
//      isn't a quad: usually merged with dark background, or partly hidden),
//      or if the blob's area is implausible for the quad. A tag36h11 black
//      region covers between the border ring alone (28/64 of the square) and
//      54/64.
//
// Coordinates are pixel centers (x + 0.5, y + 0.5) of the thresholded image.
// This model uses doubles. The RTL version's fixed-point widths come from the
// error budget in Stage 4.
#pragma once

#include <array>
#include <optional>

#include "gv/model/components.h"

namespace gv::model {

struct QuadParams {
  double merge_distance = 1.5;  // pixels
  double max_outside = 2.0;     // pixels, or this fraction of the mean side if larger
                                // (1.5 sent 18% of 20-40 px tags to 4C; 2.0 sends 4%):
  double max_outside_frac = 0.05;
  double min_fill = 0.25;       // blob area / quad area
  double max_fill = 0.95;
  double min_side = 8.0;  // pixels: a tag is 8 cells across, so it can't decode below this
};

enum class QuadVerdict { kOk, kTooFewCorners, kTooSmall, kExtremesOffQuad, kFillOutOfRange };

struct Quad {
  std::array<std::array<double, 2>, 4> corners{};  // in direction order (clockwise on screen)
  double area = 0;
};

struct QuadResult {
  QuadVerdict verdict = QuadVerdict::kTooFewCorners;
  Quad quad;  // valid unless verdict is kTooFewCorners
};

QuadResult scorecard_quad(const Component& c, const QuadParams& params = {});

const char* to_string(QuadVerdict v);

}  // namespace gv::model
