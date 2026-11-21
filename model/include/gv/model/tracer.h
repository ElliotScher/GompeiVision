// Quad finding 4C: the on-demand contour tracer for 4B's rejects (PLAN step
// 4C). This model is simpler than the PLAN's eventual design (angle sort,
// line fit and corner search as in 4A), and is enough to measure how many of
// 4B's rejects a tracer recovers:
//   1. Trace the component's outer boundary (Moore-neighbor tracing, by label,
//      from its first pixel in raster order, which is always on the boundary).
//   2. Take the boundary's convex hull and the 4 hull points that enclose the
//      largest area.
//   3. Accept only if the boundary follows the quad: the RMS distance of each
//      edge's boundary points from that edge is at most max_rms (pixels, or
//      that fraction of the edge length if larger). A quad with bumps passes;
//      an L shape or a tag merged with background doesn't.
// The PL version walks the blob in the on-chip strip buffer, one blob at a
// time, capped at a few candidates per frame.
#pragma once

#include <optional>
#include <vector>

#include "gv/model/components.h"
#include "gv/model/quad.h"

namespace gv::model {

struct TracerParams {
  double max_rms = 1.0;        // px
  double max_rms_frac = 0.03;  // of the edge length
  double min_side = 8.0;       // px, as 4B
  size_t max_boundary = 20000; // give up on huge blobs
};

// The outer boundary of component `id` (pixel centers, in tracing order).
std::vector<std::array<double, 2>> trace_boundary(const Labeling& l, uint32_t id, size_t max_points = 20000);

std::optional<Quad> trace_quad(const Labeling& l, const Component& c, const TracerParams& p = {});

}  // namespace gv::model
