// Streaming connected-components golden model (PLAN step 3).
//
// The labeler sees the thresholded image one row at a time, the way the RTL
// will. It keeps only the previous row's labels, an equivalence table and one
// feature accumulator per component, and it reports each component as soon as
// a row adds no pixels to it ("closes"), before the rest of the frame arrives.
//
// Which pixels join is defined by connected_components() in AprilTag v3.4.5
// (do_unionfind_first_line / do_unionfind_line2), and the partition must match
// it exactly. The rules, for a "reference" pixel (x, y) with value v != 127
// and 1 <= x <= w - 2:
//   - join the left neighbor (x-1, y) if equal;
//   - y > 0: join the pixel above (x, y-1) if equal;
//   - y > 0 and v == 255 (white is 8-connected, black 4-connected): join
//     (x-1, y-1) and (x+1, y-1) if equal.
// AprilTag also skips some of these joins when a neighbor pattern means
// they're already connected. That's harmless inside the image, but at the
// edges it changes the result, because columns 0 and w-1 are never reference
// pixels. For example, (w-2, y-1) and (w-1, y-1) are never joined directly,
// and the reference skips the up-right join when (x, y-1) == (x+1, y-1). So
// the model applies exactly the same skips, and the RTL must as well.
// Pixels equal to 127 are never joined to anything.
#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "gv/model/image.h"

namespace gv::model {

// Directions for the 4B corner scorecard (PLAN step 4B): K = 16 integer
// vectors, round(64 * (cos, sin)) of k * 22.5 degrees. For each direction the
// accumulator keeps the pixel with the largest dx * x + dy * y; ties keep the
// pixel first in raster order. Image y points down.
inline constexpr int kDirections = 16;
inline constexpr std::array<std::array<int, 2>, kDirections> kDirectionVectors{{
    {64, 0}, {59, 24}, {45, 45}, {24, 59}, {0, 64}, {-24, 59}, {-45, 45}, {-59, 24},
    {-64, 0}, {-59, -24}, {-45, -45}, {-24, -59}, {0, -64}, {24, -59}, {45, -45}, {59, -24}}};

struct Extreme {
  int x = 0, y = 0;
  int32_t score = 0;
  bool operator==(const Extreme&) const = default;
};

// One closed component's feature accumulator. Every field merges with min,
// max or sum, so joining two components just merges their accumulators.
struct Component {
  uint32_t id = 0;       // raster index (y * w + x) of its first pixel
  uint8_t color = 0;     // 0 (black) or 255 (white)
  uint32_t area = 0;     // pixel count
  int x_min = 0, x_max = 0, y_min = 0, y_max = 0;
  int closed_after_row = 0;  // the row whose end closed it (y_max + 1, or h - 1 at frame end)
  std::array<Extreme, kDirections> extremes{};  // farthest pixel along each direction
  bool operator==(const Component&) const = default;
};

struct Labeling {
  int width = 0;
  int height = 0;
  // Per pixel: the id of its component. A 127 pixel gets its own raster index
  // (a singleton, as in the reference union-find).
  std::vector<uint32_t> label;
  // Components (127 excluded) in the order they closed. Within one row they
  // come in left-to-right order of their leftmost pixel in the row above;
  // at frame end, in left-to-right order of the last row.
  std::vector<Component> closed;
};

Labeling label_components(const Image8& thresholded);

// Which of (left, up, up-left, up-right) the reference tries to join for the
// reference pixel (x, y), before the equality test. Exposed for the RTL tests.
struct JoinMask {
  bool left = false, up = false, up_left = false, up_right = false;
};
JoinMask reference_joins(const Image8& thresholded, int x, int y);

// Accumulator update rules, shared by the labeler and anything that rebuilds
// accumulators (the RTL testbenches). `width` orders ties by raster index.
Component new_component(uint32_t id, uint8_t color, int x, int y);
void add_pixel(Component& c, int x, int y);
void merge(Component& into, const Component& from, int width);

}  // namespace gv::model
