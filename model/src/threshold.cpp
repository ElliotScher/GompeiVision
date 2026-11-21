#include "gv/model/threshold.h"

#include <algorithm>
#include <stdexcept>

namespace gv::model {

TileStats tile_minmax(const Image8& in) {
  if (in.width < kTileSize || in.height < kTileSize)
    throw std::invalid_argument("threshold: image must be at least 4x4");
  TileStats s;
  s.tiles_x = in.width / kTileSize;
  s.tiles_y = in.height / kTileSize;
  const auto n = static_cast<size_t>(s.tiles_x * s.tiles_y);
  s.min.assign(n, 255);
  s.max.assign(n, 0);
  for (int ty = 0; ty < s.tiles_y; ++ty)
    for (int tx = 0; tx < s.tiles_x; ++tx) {
      const auto i = static_cast<size_t>(ty * s.tiles_x + tx);
      for (int dy = 0; dy < kTileSize; ++dy)
        for (int dx = 0; dx < kTileSize; ++dx) {
          const uint8_t v = in.at(tx * kTileSize + dx, ty * kTileSize + dy);
          s.min[i] = std::min(s.min[i], v);
          s.max[i] = std::max(s.max[i], v);
        }
    }
  return s;
}

TileStats dilate_tiles(const TileStats& raw) {
  TileStats d = raw;
  for (int ty = 0; ty < raw.tiles_y; ++ty)
    for (int tx = 0; tx < raw.tiles_x; ++tx) {
      uint8_t mn = 255, mx = 0;
      for (int ny = std::max(ty - 1, 0); ny <= std::min(ty + 1, raw.tiles_y - 1); ++ny)
        for (int nx = std::max(tx - 1, 0); nx <= std::min(tx + 1, raw.tiles_x - 1); ++nx) {
          mn = std::min(mn, raw.min_at(nx, ny));
          mx = std::max(mx, raw.max_at(nx, ny));
        }
      const auto i = static_cast<size_t>(ty * raw.tiles_x + tx);
      d.min[i] = mn;
      d.max[i] = mx;
    }
  return d;
}

Image8 binarize(const Image8& in, const TileStats& dilated, const ThresholdParams& params) {
  Image8 out(in.width, in.height);
  const int full_w = dilated.tiles_x * kTileSize;
  const int full_h = dilated.tiles_y * kTileSize;
  for (int y = 0; y < in.height; ++y) {
    const int ty = std::min(y / kTileSize, dilated.tiles_y - 1);
    for (int x = 0; x < in.width; ++x) {
      const int tx = std::min(x / kTileSize, dilated.tiles_x - 1);
      const int mn = dilated.min_at(tx, ty);
      const int mx = dilated.max_at(tx, ty);
      const bool full_tile = x < full_w && y < full_h;
      if (full_tile && mx - mn < params.min_white_black_diff) {
        out.at(x, y) = kSkip;
        continue;
      }
      const int thresh = mn + (mx - mn) / 2;
      out.at(x, y) = in.at(x, y) > thresh ? 255 : 0;
    }
  }
  return out;
}

ThresholdResult threshold(const Image8& in, const ThresholdParams& params) {
  ThresholdResult r;
  r.raw = tile_minmax(in);
  r.dilated = dilate_tiles(r.raw);
  r.image = binarize(in, r.dilated, params);
  return r;
}

}  // namespace gv::model
