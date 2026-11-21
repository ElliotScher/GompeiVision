#include "gv/model/image.h"

#include <cctype>
#include <fstream>
#include <stdexcept>
#include <string>

namespace gv::model {

namespace {

// Reads the next whitespace-separated header token, skipping # comments.
std::string next_token(std::istream& in) {
  std::string tok;
  while (in) {
    int c = in.get();
    if (c == '#') {
      while (in && c != '\n') c = in.get();
    } else if (std::isspace(c)) {
      if (!tok.empty()) return tok;
    } else if (c != EOF) {
      tok.push_back(static_cast<char>(c));
    }
  }
  return tok;
}

}  // namespace

Image8 read_pgm(const std::filesystem::path& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) throw std::runtime_error("cannot open " + path.string());
  if (next_token(in) != "P5") throw std::runtime_error(path.string() + ": not a binary PGM (P5)");
  const int w = std::stoi(next_token(in));
  const int h = std::stoi(next_token(in));
  const int maxval = std::stoi(next_token(in));
  if (w <= 0 || h <= 0 || maxval != 255)
    throw std::runtime_error(path.string() + ": need positive size and maxval 255");
  // next_token consumed the single whitespace byte after maxval.
  Image8 img(w, h);
  in.read(reinterpret_cast<char*>(img.pixels.data()), static_cast<std::streamsize>(img.pixels.size()));
  if (!in) throw std::runtime_error(path.string() + ": truncated pixel data");
  return img;
}

void write_pgm(const std::filesystem::path& path, const Image8& image) {
  std::ofstream out(path, std::ios::binary);
  if (!out) throw std::runtime_error("cannot write " + path.string());
  out << "P5\n" << image.width << ' ' << image.height << "\n255\n";
  out.write(reinterpret_cast<const char*>(image.pixels.data()),
            static_cast<std::streamsize>(image.pixels.size()));
}

void write_ppm(const std::filesystem::path& path, int width, int height,
               const std::vector<uint8_t>& rgb) {
  if (rgb.size() != static_cast<size_t>(width) * static_cast<size_t>(height) * 3)
    throw std::invalid_argument("write_ppm: rgb size does not match width * height * 3");
  std::ofstream out(path, std::ios::binary);
  if (!out) throw std::runtime_error("cannot write " + path.string());
  out << "P6\n" << width << ' ' << height << "\n255\n";
  out.write(reinterpret_cast<const char*>(rgb.data()), static_cast<std::streamsize>(rgb.size()));
}

}  // namespace gv::model
