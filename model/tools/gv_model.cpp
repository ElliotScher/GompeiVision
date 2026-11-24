// gv-model: run a frame through the golden models and dump every
// intermediate, for comparing against RTL simulation dumps and for eyeballing.
//
//   gv-model dump INPUT.pgm OUTDIR [--decimate N] [--min-white-black-diff D]
//   gv-model replay SESSION_DIR [--decimate N] [--csv OUT.csv]
//   gv-model edges INPUT.pgm OUT.csv
//   gv-model synth OUT.pgm [--seed N] [--size W H] [--tags N] [--tag-px MIN MAX]
//   gv-model lookup-tables OUTDIR
//   gv-model lookup [--max-bits N] < codes
//   gv-model decodes INPUT.pgm OUT.csv
//   gv-model project H0 .. H7
//   gv-model quads < records
//   gv-model read INPUT.pgm < coefficients
//   gv-model sample INPUT.pgm < jobs
//   gv-model detect INPUT.pgm
//
// detect prints the golden detector's tags at the Stage 4 widths, without 4C
// (what PL and RPU must reproduce), one per line: "id hamming margin x 16 cut",
// then the corners x 256, "x0 y0 .. x3 y3".
//
// sample runs the PL edge sampler on arbitrary jobs, one per line with the
// fields of edges' columns (a_x .. check_direction, same units). It prints each
// job's points as "x:y" pairs on one line, or "-" for none.
//
// read runs the PL projector, tag sampler and ID lookup on homographies, one
// per line as 8 coefficients (as for project): "invalid", or the read's ok,
// code (hex) and margin x 16, then "id rotation hamming" or "-".
//
// quads runs the 4B corner scorecard on component records, one per line:
// "area x_min x_max y_min y_max ex0 ey0 .. ex15 ey15". It prints the verdict
// (0 ok, 1 too few corners, 2 too small, 3 extremes off quad, 4 fill out of
// range), then unless it's 1 the 4 corners' pixel indices "x0 y0 .. x3 y3" and
// twice the quad's area.
//
// decodes writes every code read the detector makes at the Stage 4 widths
// (the PL projector's and tag sampler's work): whether the projector is valid,
// its 8 coefficients (h0..h5 x 2^14, h6, h7 x 2^24), 64 border and 36 cell
// sample points (x 256, "x:y" separated by spaces), then ok, the code in hex
// and the decision margin x 16.
//
// project runs the PL projector on 8 coefficients as the RPU sends them
// (h0..h5 x 2^14, h6, h7 x 2^24): "valid" and the 100 points (x 256, "x:y"
// per line), or "invalid".
//
// lookup-tables writes the ID lookup's ROMs as $readmemh files: codes.mem
// (the 2,348 rotated codes, index = rotation * 587 + id), near.mem (36
// touching-cell masks), and per 9-bit block b: start_b.mem (513 bucket
// offsets) and list_b.mem (rotated-code indices by bucket). lookup reads one
// hex code per line and prints the golden decode: "id rotation hamming", or
// "-" for no match.
//
// synth renders a synthetic tag36h11 scene (tests/synth.h) for testbenches.
//
// edges runs the detector at the Stage 4 fixed-point widths and writes every
// edge job the PL edge sampler gets, with the edge points it must return, in
// integer fixed-point units: coordinates x 256, directions x 4096. One row per
// job; `points` is "x:y" pairs separated by spaces.
//
// replay runs the golden detector over a recorded session (model/datasets/)
// and diffs it against the CPU pipeline's detections; --decimate defaults to
// the session's quad_decimate. Exits 1 if any frame disagrees.
//
// dump
// Writes to OUTDIR:
//   decimated.pgm               input after decimation (skipped when N = 1)
//   tile_min.pgm, tile_max.pgm  raw 4x4 tile statistics (one pixel per tile)
//   tile_min_dilated.pgm, tile_max_dilated.pgm
//   threshold.pgm               0 / 127 / 255
//   components.ppm              each component in a pseudo-random color
//   components.csv              closed components, in closure order, with the
//                               4B scorecard verdict and rough corners, then the
//                               16 extreme pixels (ex0, ey0 ... ex15, ey15)
#include <cmath>
#include <cstdio>
#include <functional>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "gv/model/components.h"
#include "gv/model/dataset.h"
#include "gv/model/detector.h"
#include "gv/model/fixed_point.h"
#include "gv/model/refine.h"
#include "gv/model/tag_lookup.h"
#include "gv/model/decimate.h"
#include "gv/model/quad.h"
#include "gv/model/threshold.h"
#include "synth.h"

namespace fs = std::filesystem;
using namespace gv::model;

namespace {

int usage() {
  std::cerr << "usage: gv-model dump INPUT.pgm OUTDIR [--decimate N] [--min-white-black-diff D]\n"
               "       gv-model replay SESSION_DIR [--decimate N] [--csv OUT.csv]\n"
               "       gv-model edges INPUT.pgm OUT.csv\n"
               "       gv-model synth OUT.pgm [--seed N] [--size W H] [--tags N] [--tag-px MIN MAX]\n"
               "       gv-model lookup-tables OUTDIR\n"
               "       gv-model lookup [--max-bits N] < codes\n"
               "       gv-model decodes INPUT.pgm OUT.csv\n"
               "       gv-model project H0 .. H7\n"
               "       gv-model quads < records\n"
               "       gv-model read INPUT.pgm < coefficients\n"
               "       gv-model sample INPUT.pgm < jobs\n"
               "       gv-model detect INPUT.pgm\n";
  return 2;
}

int lookup_tables_main(int argc, char** argv) {
  if (argc != 3) return usage();
  const fs::path dir = argv[2];
  fs::create_directories(dir);
  const Tag36h11Lookup l;
  auto hex = [](std::ofstream& f, uint64_t v, int digits) {
    char buf[24];
    std::snprintf(buf, sizeof buf, "%0*llx\n", digits, static_cast<unsigned long long>(v));
    f << buf;
  };
  std::ofstream codes(dir / "codes.mem"), near(dir / "near.mem");
  for (int i = 0; i < l.num_rotated(); ++i) hex(codes, l.rotated(i), 9);
  for (int k = 0; k < Tag36h11Lookup::kBits; ++k) hex(near, l.near_mask(k), 9);
  // Each bit's cell (x, y), one hex digit each: the tag sampler's layout ROM.
  std::ofstream cells(dir / "cells.mem");
  for (int k = 0; k < Tag36h11Lookup::kBits; ++k) hex(cells, static_cast<uint64_t>(l.cell_x(k) * 16 + l.cell_y(k)), 2);
  for (int b = 0; b < Tag36h11Lookup::kBlocks; ++b) {
    std::ofstream start(dir / ("start_" + std::to_string(b) + ".mem")), list(dir / ("list_" + std::to_string(b) + ".mem"));
    uint32_t offset = 0;
    for (uint32_t v = 0; v < 512; ++v) {
      hex(start, offset, 4);
      for (uint32_t i : l.bucket(b, v)) hex(list, i, 3);
      offset += static_cast<uint32_t>(l.bucket(b, v).size());
    }
    hex(start, offset, 4);
  }
  std::cout << "wrote " << l.num_rotated() << " codes, max bucket " << l.max_bucket_size() << ", to " << dir.string() << "\n";
  return 0;
}

int decodes_main(int argc, char** argv) {
  if (argc != 4) return usage();
  try {
    const Image8 img = read_pgm(argv[2]);
    std::ofstream csv(argv[3]);
    csv << "projected,h,border,cells,ok,code,margin\n";
    auto pts = [](const auto& list) {
      std::string out;
      for (const auto& p : list) {
        if (!out.empty()) out += ' ';
        out += std::to_string(std::llround(p[0] * 256)) + ":" + std::to_string(std::llround(p[1] * 256));
      }
      return out;
    };
    int reads = 0;
    const DecodeTrace trace = [&](const DecodeSample& d) {
      ++reads;
      char code[16];
      std::snprintf(code, sizeof code, "%09llx", static_cast<unsigned long long>(d.code));
      std::string h;
      for (int64_t v : d.proj_a) h += std::to_string(v) + ' ';
      h += std::to_string(d.proj_g[0]) + ' ' + std::to_string(d.proj_g[1]);
      csv << int{d.projected} << ',' << h << ',' << pts(d.border) << ',' << pts(d.cells) << ',' << int{d.ok} << ',' << code << ','
          << std::llround(d.decision_margin * 16) << '\n';
    };
    static const Tag36h11Lookup family;
    DetectorParams dp;
    dp.quant = kStage4Widths;
    dp.decode_trace = &trace;
    const auto dets = detect(img, family, dp);
    std::cout << img.width << "x" << img.height << ": " << reads << " code reads, " << dets.size() << " tags\n";
  } catch (const std::exception& e) {
    std::cerr << "gv-model: " << e.what() << "\n";
    return 1;
  }
  return 0;
}

int project_main(int argc, char** argv) {
  if (argc != 10) return usage();
  const Quantization q = kStage4Widths;
  Homography h{};
  for (int i = 0; i < 8; ++i)
    h[static_cast<size_t>(i)] = std::ldexp(static_cast<double>(std::stoll(argv[2 + i])), i < 6 ? -q.proj_scale_frac : -q.proj_persp_frac);
  h[8] = 1;
  const FixedProjector fp(h, q);
  std::cout << (fp.valid ? "valid" : "invalid") << "\n";
  if (!fp.valid) return 0;
  static const Tag36h11Lookup family;
  for (const auto& [U, V] : decode_sample_grid(family))
    std::cout << fp.coord(0, U, V) << ":" << fp.coord(1, U, V) << "\n";
  return 0;
}

int quads_main(int argc) {
  if (argc != 2) return usage();
  std::string line;
  while (std::getline(std::cin, line)) {
    if (line.empty()) continue;
    std::istringstream in(line);
    Component c;
    in >> c.area >> c.x_min >> c.x_max >> c.y_min >> c.y_max;
    for (Extreme& e : c.extremes) in >> e.x >> e.y;
    if (!in) {
      std::cerr << "gv-model quads: bad record: " << line << "\n";
      return 1;
    }
    const QuadResult r = scorecard_quad(c);
    std::cout << static_cast<int>(r.verdict);
    if (r.verdict != QuadVerdict::kTooFewCorners) {
      for (const auto& p : r.quad.corners)
        std::cout << ' ' << std::llround(p[0] - 0.5) << ' ' << std::llround(p[1] - 0.5);
      std::cout << ' ' << std::llround(r.quad.area * 2);
    }
    std::cout << '\n';
  }
  return 0;
}

int read_main(int argc, char** argv) {
  if (argc != 3) return usage();
  try {
    const Image8 img = read_pgm(argv[2]);
    static const Tag36h11Lookup family;
    const Quantization q = kStage4Widths;
    std::string line;
    while (std::getline(std::cin, line)) {
      if (line.empty()) continue;
      std::istringstream in(line);
      Homography h{};
      long long v = 0;
      for (int i = 0; i < 8; ++i) {
        in >> v;
        h[static_cast<size_t>(i)] = std::ldexp(static_cast<double>(v), i < 6 ? -q.proj_scale_frac : -q.proj_persp_frac);
      }
      h[8] = 1;
      if (!in) throw std::runtime_error("bad line: " + line);
      if (!FixedProjector(h, q).valid) {
        std::cout << "invalid\n";
        continue;
      }
      const auto r = read_code_homography(img, h, family, DetectorParams{}.sharpening, q);
      if (!r) {
        std::cout << "0 0 0 -\n";
        continue;
      }
      char code[16];
      std::snprintf(code, sizeof code, "%09llx", static_cast<unsigned long long>(r->code));
      std::cout << "1 " << code << ' ' << std::llround(r->decision_margin * 16);
      const auto m = family.decode(r->code, DetectorParams{}.max_hamming);
      if (m) std::cout << ' ' << m->id << ' ' << m->rotation << ' ' << m->hamming << '\n';
      else std::cout << " -\n";
    }
  } catch (const std::exception& e) {
    std::cerr << "gv-model: " << e.what() << "\n";
    return 1;
  }
  return 0;
}

int sample_main(int argc, char** argv) {
  if (argc != 3) return usage();
  try {
    const Image8 img = read_pgm(argv[2]);
    const Quantization q = kStage4Widths;
    std::string line;
    while (std::getline(std::cin, line)) {
      if (line.empty()) continue;
      std::istringstream in(line);
      long long v[12];
      for (long long& x : v) in >> x;
      if (!in) throw std::runtime_error("bad line: " + line);
      auto fx = [](long long x, int frac) { return std::ldexp(static_cast<double>(x), -frac); };
      EdgeJob j;
      j.a = {fx(v[0], q.coord_frac), fx(v[1], q.coord_frac)};
      j.d = {fx(v[2], q.direction_frac), fx(v[3], q.direction_frac)};
      j.n = {fx(v[4], q.direction_frac), fx(v[5], q.direction_frac)};
      j.t0 = fx(v[6], q.coord_frac);
      j.spacing = fx(v[7], q.coord_frac);
      j.count = static_cast<int>(v[8]);
      j.half_steps = static_cast<int>(v[9]);
      j.window_steps = static_cast<int>(v[10]);
      j.check_direction = v[11] != 0;
      std::string out;
      for (const auto& p : sample_edge(img, j, q))
        out += (out.empty() ? "" : " ") + std::to_string(std::llround(std::ldexp(p[0], q.coord_frac))) + ":" +
               std::to_string(std::llround(std::ldexp(p[1], q.coord_frac)));
      std::cout << (out.empty() ? "-" : out) << '\n';
    }
  } catch (const std::exception& e) {
    std::cerr << "gv-model: " << e.what() << "\n";
    return 1;
  }
  return 0;
}

int detect_main(int argc, char** argv) {
  if (argc != 3) return usage();
  try {
    static const Tag36h11Lookup family;
    DetectorParams dp;
    dp.quant = kStage4Widths;
    dp.use_4c = false;
    for (const auto& d : detect(read_pgm(argv[2]), family, dp)) {
      std::cout << d.id << ' ' << d.hamming << ' ' << std::llround(d.decision_margin * 16) << ' ' << int{d.cut_by_frame};
      for (const auto& c : d.corners) std::cout << ' ' << std::llround(c[0] * 256) << ' ' << std::llround(c[1] * 256);
      std::cout << '\n';
    }
  } catch (const std::exception& e) {
    std::cerr << "gv-model: " << e.what() << "\n";
    return 1;
  }
  return 0;
}

int lookup_main(int argc, char** argv) {
  int max_bits = Tag36h11Lookup::kDefaultMaxBits;
  for (int i = 2; i < argc; ++i) {
    if (std::string(argv[i]) == "--max-bits" && i + 1 < argc) max_bits = std::atoi(argv[++i]);
    else return usage();
  }
  const Tag36h11Lookup l;
  std::string line;
  while (std::getline(std::cin, line)) {
    if (line.empty()) continue;
    const auto m = l.decode(std::stoull(line, nullptr, 16), max_bits);
    if (m) std::cout << m->id << ' ' << m->rotation << ' ' << m->hamming << '\n';
    else std::cout << "-\n";
  }
  return 0;
}

int synth_main(int argc, char** argv) {
  if (argc < 3) return usage();
  gv::test::SceneParams p;
  uint32_t seed = 1;
  for (int i = 3; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--seed" && i + 1 < argc) seed = static_cast<uint32_t>(std::atoi(argv[++i]));
    else if (a == "--size" && i + 2 < argc) p.width = std::atoi(argv[++i]), p.height = std::atoi(argv[++i]);
    else if (a == "--tags" && i + 1 < argc) p.num_tags = std::atoi(argv[++i]);
    else if (a == "--tag-px" && i + 2 < argc) p.min_tag_px = std::atof(argv[++i]), p.max_tag_px = std::atof(argv[++i]);
    else return usage();
  }
  try {
    write_pgm(argv[2], gv::test::make_scene(seed, p).image);
  } catch (const std::exception& e) {
    std::cerr << "gv-model: " << e.what() << "\n";
    return 1;
  }
  return 0;
}

int edges_main(int argc, char** argv) {
  if (argc != 4) return usage();
  try {
    const Image8 img = read_pgm(argv[2]);
    std::ofstream csv(argv[3]);
    csv << "a_x,a_y,d_x,d_y,n_x,n_y,t0,spacing,count,half_steps,window_steps,check_direction,points\n";
    auto fx = [](double v, int frac) { return static_cast<long long>(std::llround(std::ldexp(v, frac))); };
    const Quantization& q = kStage4Widths;
    int jobs = 0;
    const std::function<void(const EdgeJob&, const std::vector<std::array<double, 2>>&)> trace =
        [&](const EdgeJob& j, const std::vector<std::array<double, 2>>& pts) {
          ++jobs;
          csv << fx(j.a[0], q.coord_frac) << ',' << fx(j.a[1], q.coord_frac) << ',' << fx(j.d[0], q.direction_frac) << ','
              << fx(j.d[1], q.direction_frac) << ',' << fx(j.n[0], q.direction_frac) << ','
              << fx(j.n[1], q.direction_frac) << ',' << fx(j.t0, q.coord_frac) << ',' << fx(j.spacing, q.coord_frac)
              << ',' << j.count << ',' << j.half_steps << ',' << j.window_steps << ',' << int{j.check_direction} << ',';
          for (size_t i = 0; i < pts.size(); ++i)
            csv << (i ? " " : "") << fx(pts[i][0], q.coord_frac) << ':' << fx(pts[i][1], q.coord_frac);
          csv << '\n';
        };
    static const Tag36h11Lookup family;
    DetectorParams dp;
    dp.quant = q;
    dp.refine.trace = &trace;
    const auto dets = detect(img, family, dp);
    std::cout << img.width << "x" << img.height << ": " << jobs << " edge jobs, " << dets.size() << " tags\n";
  } catch (const std::exception& e) {
    std::cerr << "gv-model: " << e.what() << "\n";
    return 1;
  }
  return 0;
}

int replay_main(int argc, char** argv) {
  const fs::path dir = argv[2];
  int factor = 0;
  std::string csv_path;
  for (int i = 3; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--decimate" && i + 1 < argc) factor = std::atoi(argv[++i]);
    else if (a == "--csv" && i + 1 < argc) csv_path = argv[++i];
    else return usage();
  }
  try {
    const Dataset d = load_dataset(dir);
    DetectorParams dp;
    dp.decimate = factor > 0 ? factor : d.quad_decimate;
    std::ofstream csv;
    if (!csv_path.empty()) {
      csv.open(csv_path);
      csv << "file,timestamp,cpu_tags,golden_tags,matched,only_cpu,only_golden,worst_corner_diff_px\n";
    }
    int bad_frames = 0;
    const ReplayStats st = replay(d, dp, 2.0, [&](const ReplayFrame& f) {
      bad_frames += f.only_cpu + f.only_golden > 0;
      if (csv.is_open())
        csv << f.frame->file << ',' << f.frame->timestamp << ',' << f.frame->cpu.size() << ',' << f.golden.size() << ','
            << f.matched << ',' << f.only_cpu << ',' << f.only_golden << ',' << f.worst_diff << '\n';
    });
    std::cout << dir.string() << " (" << d.hardware_id << " " << d.role << ", AprilTag " << d.apriltag_version
              << ", decimate " << dp.decimate << ")\n"
              << "  frames " << st.frames << ": CPU " << st.cpu_tags << " tags, golden " << st.golden_tags << "\n"
              << "  matched " << st.matched << ", only CPU " << st.only_cpu << ", only golden " << st.only_golden
              << ", frames that disagree " << bad_frames << "\n"
              << "  corner difference on matched tags: median " << st.quantile(0.5) << " px, p95 "
              << st.quantile(0.95) << " px, max " << st.quantile(1.0) << " px\n";
    return bad_frames ? 1 : 0;
  } catch (const std::exception& e) {
    std::cerr << "gv-model: " << e.what() << "\n";
    return 2;
  }
}

Image8 tiles_to_image(const TileStats& s, bool want_max) {
  Image8 img(s.tiles_x, s.tiles_y);
  img.pixels = want_max ? s.max : s.min;
  return img;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc >= 3 && std::string(argv[1]) == "replay") return replay_main(argc, argv);
  if (argc >= 2 && std::string(argv[1]) == "edges") return edges_main(argc, argv);
  if (argc >= 2 && std::string(argv[1]) == "synth") return synth_main(argc, argv);
  if (argc >= 2 && std::string(argv[1]) == "lookup-tables") return lookup_tables_main(argc, argv);
  if (argc >= 2 && std::string(argv[1]) == "lookup") return lookup_main(argc, argv);
  if (argc >= 2 && std::string(argv[1]) == "decodes") return decodes_main(argc, argv);
  if (argc >= 2 && std::string(argv[1]) == "project") return project_main(argc, argv);
  if (argc >= 2 && std::string(argv[1]) == "detect") return detect_main(argc, argv);
  if (argc >= 2 && std::string(argv[1]) == "sample") return sample_main(argc, argv);
  if (argc >= 2 && std::string(argv[1]) == "read") return read_main(argc, argv);
  if (argc >= 2 && std::string(argv[1]) == "quads") return quads_main(argc);
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
    csv << "id,color,area,x_min,x_max,y_min,y_max,closed_after_row,quad_4b,x0,y0,x1,y1,x2,y2,x3,y3";
    for (int k = 0; k < kDirections; ++k) csv << ",ex" << k << ",ey" << k;
    csv << "\n";
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
      for (const Extreme& e : c.extremes) csv << ',' << e.x << ',' << e.y;
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
