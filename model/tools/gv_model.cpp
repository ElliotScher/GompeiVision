// gv-model: run a frame through the golden models and dump every
// intermediate, for comparing against RTL simulation dumps and for eyeballing.
//
//   gv-model dump INPUT.pgm OUTDIR [--decimate N] [--min-white-black-diff D]
//
// Writes to OUTDIR:
//   decimated.pgm               input after decimation (skipped when N = 1)
//   tile_min.pgm, tile_max.pgm  raw 4x4 tile statistics (one pixel per tile)
//   tile_min_dilated.pgm, tile_max_dilated.pgm
//   threshold.pgm               0 / 127 / 255
//   components.ppm              each component in a pseudo-random color
//   components.csv              closed components, in closure order, with the
//                               4B scorecard verdict and rough corners
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "gv/model/components.h"
#include "gv/model/decimate.h"
#include "gv/model/quad.h"
#include "gv/model/threshold.h"

namespace fs = std::filesystem;
using namespace gv::model;

namespace {

int usage() {
  std::cerr << "usage: gv-model dump INPUT.pgm OUTDIR [--decimate N] [--min-white-black-diff D]\n";
  return 2;
}

Image8 tiles_to_image(const TileStats& s, bool want_max) {
  Image8 img(s.tiles_x, s.tiles_y);
  img.pixels = want_max ? s.max : s.min;
  return img;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 4 || std::string(argv[1]) != "dump") return usage();
  const fs::path input = argv[2], outdir = argv[3];
  int factor = 1;
  ThresholdParams params;
  for (int i = 4; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--decimate" && i + 1 < argc) factor = std::atoi(argv[++i]);
    else if (a == "--min-white-black-diff" && i + 1 < argc) params.min_white_black_diff = std::atoi(argv[++i]);
    else return usage();
  }
  try {
    fs::create_directories(outdir);
    Image8 img = read_pgm(input);
    if (factor != 1) {
      img = decimate(img, factor);
      write_pgm(outdir / "decimated.pgm", img);
    }
    const ThresholdResult t = threshold(img, params);
    write_pgm(outdir / "tile_min.pgm", tiles_to_image(t.raw, false));
    write_pgm(outdir / "tile_max.pgm", tiles_to_image(t.raw, true));
    write_pgm(outdir / "tile_min_dilated.pgm", tiles_to_image(t.dilated, false));
    write_pgm(outdir / "tile_max_dilated.pgm", tiles_to_image(t.dilated, true));
    write_pgm(outdir / "threshold.pgm", t.image);

    const Labeling l = label_components(t.image);
    std::vector<uint8_t> rgb(l.label.size() * 3, 127);
    for (size_t i = 0; i < l.label.size(); ++i) {
      if (t.image.pixels[i] == kSkip) continue;
      const uint32_t hsh = l.label[i] * 2654435761u;
      rgb[3 * i] = static_cast<uint8_t>(hsh >> 24);
      rgb[3 * i + 1] = static_cast<uint8_t>(hsh >> 16);
      rgb[3 * i + 2] = static_cast<uint8_t>(hsh >> 8);
    }
    write_ppm(outdir / "components.ppm", l.width, l.height, rgb);
    std::ofstream csv(outdir / "components.csv");
    csv << "id,color,area,x_min,x_max,y_min,y_max,closed_after_row,quad_4b,x0,y0,x1,y1,x2,y2,x3,y3\n";
    for (const Component& c : l.closed) {
      csv << c.id << ',' << int{c.color} << ',' << c.area << ',' << c.x_min << ',' << c.x_max << ','
          << c.y_min << ',' << c.y_max << ',' << c.closed_after_row;
      const QuadResult q = scorecard_quad(c);
      if (c.color == 0 && q.verdict != QuadVerdict::kTooFewCorners) {
        csv << ',' << to_string(q.verdict);
        for (const auto& p : q.quad.corners) csv << ',' << p[0] << ',' << p[1];
      } else {
        csv << ",,,,,,,,,";
      }
      csv << '\n';
    }
    std::cout << img.width << "x" << img.height << ": " << l.closed.size() << " components, dumps in "
              << outdir.string() << "\n";
  } catch (const std::exception& e) {
    std::cerr << "gv-model: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
