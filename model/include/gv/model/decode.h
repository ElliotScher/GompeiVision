// Decode golden model (PLAN step 6): bit sampling and the brightness model,
// following quad_decode() in AprilTag v3.4.5. The ID lookup is
// Tag36h11Lookup (NAPA's method).
//
//   1. Homography: Heckbert's closed-form square-to-quad mapping from tag
//      coordinates (-1..1, corners (-1,-1), (1,-1), (1,1), (-1,1)) to the quad's
//      corners, in the quad's order (clockwise on screen).
//   2. Brightness model: least-squares planes v = a*x + b*y + c (in tag
//      coordinates) through samples of the white ring just outside the black
//      border and of the black border itself, 8 samples per side each, read
//      from the pixel containing each sample point.
//   3. Bits: bilinear value at each data cell's center, minus the midpoint of
//      the two planes there, then sharpened on the 10x10 cell grid with a
//      Laplacian kernel weighted by `sharpening` (AprilTag's decode_sharpening).
//      Bit = value > 0.
//   4. Decision margin, as AprilTag: the smaller of the mean |value| of the
//      white bits and of the black bits (each count starting at 1).
// The model uses doubles; Stage 4's fixed-point version keeps the same steps.
#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <optional>

#include "gv/model/fixed_point.h"
#include "gv/model/image.h"
#include "gv/model/quad.h"
#include "gv/model/tag_lookup.h"

namespace gv::model {

using Homography = std::array<double, 9>;  // row-major, maps (u, v, 1) in the unit square

Homography square_to_quad(const std::array<std::array<double, 2>, 4>& quad);
std::array<double, 2> project(const Homography& h, double u, double v);
// Tag coordinates (-1..1) to image coordinates.
std::array<double, 2> project_tag(const Homography& h, double tag_x, double tag_y);

// The PL projector's arithmetic (Quantization::fixed_projector()): the
// homography in 32-bit fixed point, a[] = h0..h5 with scale_frac fractional
// bits and g[] = h6, h7 with persp_frac (h8 = 1). Sample point (U, V), on the
// 1/16 grid of the unit square (U = 16u, -1..17), maps to
//   x = round(num_x * 2^(persp_frac + out_frac - scale_frac) / den)
//   num_x = a0 U + a1 V + 16 a2,  den = g0 U + g1 V + 16 * 2^persp_frac
// (y likewise with a3..a5), one exact division rounded half away from zero to
// out_frac bits. Valid (the RPU sends it, else the read fails) when the
// coefficients fit, den > 0 over the samples and every point is within
// +-32768 px. Bit-exact reference for rtl/decode/projector.sv.
__extension__ using int128 = __int128;

// The 100 sample points of a read in the order the tag sampler takes them
// (64 border samples, AprilTag's 8 patterns x 8, then the 36 cells in the
// family's bit order), as (U, V) = 16 (u, v) on the unit square.
std::array<std::array<int, 2>, 100> decode_sample_grid(const Tag36h11Lookup& family);
struct FixedProjector {
  FixedProjector(const Homography& h, const Quantization& q);
  std::array<double, 2> project_tag(double tag_x, double tag_y) const;  // tag coordinates on the 1/8 grid
  int128 num(int c, int U, int V) const;  // c: 0 = x, 1 = y
  int128 den(int U, int V) const;
  int64_t coord(int c, int U, int V) const;  // out_frac fractional bits
  bool valid = false;
  int64_t a[6] = {}, g[2] = {};
  int scale_frac, persp_frac, out_frac;
};

struct CodeRead {
  uint64_t code = 0;          // 36 bits, MSB first in the family's cell order
  double decision_margin = 0;
};

// One read's sample points and result, for the PL tag sampler's testbench:
// the 64 border samples (8 patterns x 8, AprilTag's order) and the 36 cell
// centers (the family's bit order), as projected (coord_frac bits), and the
// code and margin, or ok = false.
struct DecodeSample {
  std::array<std::array<double, 2>, 4> quad{};  // the corners read from
  // The fixed-point projector's coefficients, and whether it was valid (else
  // no points: the read failed before sampling).
  bool projected = true;
  std::array<int64_t, 6> proj_a{};
  std::array<int64_t, 2> proj_g{};
  std::array<std::array<double, 2>, 64> border{};
  std::array<std::array<double, 2>, 36> cells{};
  bool ok = false;
  uint64_t code = 0;
  double decision_margin = 0;
};
using DecodeTrace = std::function<void(const DecodeSample&)>;

// nullopt if the border is reversed (inside brighter than outside) or the
// quad leaves the image.
std::optional<CodeRead> read_code(const Image8& image, const std::array<std::array<double, 2>, 4>& quad,
                                  const Tag36h11Lookup& family, double sharpening = 0.25,
                                  const Quantization& quant = {}, const DecodeTrace* trace = nullptr);

// The same from a homography (unit square to image), for testbenches.
std::optional<CodeRead> read_code_homography(const Image8& image, const Homography& h, const Tag36h11Lookup& family,
                                             double sharpening = 0.25, const Quantization& quant = {});

}  // namespace gv::model
