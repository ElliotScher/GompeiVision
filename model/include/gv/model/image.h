// 8-bit grayscale image used by every golden model, plus PGM/PPM file I/O for
// the dump tool and datasets.
#pragma once

#include <cstdint>
#include <filesystem>
#include <vector>

namespace gv::model {

struct Image8 {
  int width = 0;
  int height = 0;
  std::vector<uint8_t> pixels;  // row-major, stride == width

  Image8() = default;
  Image8(int w, int h, uint8_t fill = 0)
      : width(w), height(h), pixels(static_cast<size_t>(w) * static_cast<size_t>(h), fill) {}

  uint8_t& at(int x, int y) { return pixels[index(x, y)]; }
  uint8_t at(int x, int y) const { return pixels[index(x, y)]; }
  size_t index(int x, int y) const {
    return static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x);
  }
  bool operator==(const Image8&) const = default;
};

// Binary PGM (P5, maxval 255). Throws std::runtime_error on bad input.
Image8 read_pgm(const std::filesystem::path& path);
void write_pgm(const std::filesystem::path& path, const Image8& image);

// Binary PPM (P6), interleaved RGB, width * height * 3 bytes.
void write_ppm(const std::filesystem::path& path, int width, int height,
               const std::vector<uint8_t>& rgb);

}  // namespace gv::model
