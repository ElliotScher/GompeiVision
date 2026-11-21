// Adaptive threshold golden model (PLAN step 2), bit-exact with threshold()
// in AprilTag v3.4.5 (apriltag_quad_thresh.c) with deglitch off, which is the
// library default and what the CPU pipeline uses.
//
// The steps, each kept as an intermediate the RTL is checked against:
//   1. Tile statistics: min and max over each full 4x4 tile. Partial tiles on
//      the right and bottom edges get no statistics of their own.
//   2. Dilation: each tile's min/max becomes the min/max over its 3x3
//      neighborhood of tiles, clipped at the grid edge.
//   3. Binarize. Pixels in full tiles: 127 ("no edges, skip") if
//      max - min < min_white_black_diff, else 255 if v > min + (max - min) / 2,
//      else 0. Pixels in the partial edge tiles use the nearest full tile's
//      dilated statistics and are ALWAYS binarized, never 127. That is a quirk
//      of the reference, and the RTL must match it.
//
// Images must be at least 4x4 (one full tile). The reference reads out of
// bounds below that.
#pragma once

#include <cstdint>
#include <vector>

#include "gv/model/image.h"

namespace gv::model {

inline constexpr int kTileSize = 4;
inline constexpr uint8_t kSkip = 127;  // low-contrast pixel, ignored downstream

struct ThresholdParams {
  int min_white_black_diff = 5;  // AprilTag default (td->qtp.min_white_black_diff)
};

// Per-tile statistics on a tiles_x by tiles_y grid (full tiles only).
struct TileStats {
  int tiles_x = 0;
  int tiles_y = 0;
  std::vector<uint8_t> min;  // row-major, tiles_x * tiles_y
  std::vector<uint8_t> max;

  uint8_t min_at(int tx, int ty) const { return min[static_cast<size_t>(ty * tiles_x + tx)]; }
  uint8_t max_at(int tx, int ty) const { return max[static_cast<size_t>(ty * tiles_x + tx)]; }
  bool operator==(const TileStats&) const = default;
};

TileStats tile_minmax(const Image8& in);
TileStats dilate_tiles(const TileStats& raw);
Image8 binarize(const Image8& in, const TileStats& dilated, const ThresholdParams& params);

struct ThresholdResult {
  TileStats raw;      // step 1
  TileStats dilated;  // step 2
  Image8 image;       // step 3: 0, 255 or kSkip per pixel
};

ThresholdResult threshold(const Image8& in, const ThresholdParams& params = {});

}  // namespace gv::model
