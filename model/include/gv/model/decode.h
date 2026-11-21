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
#include <optional>

#include "gv/model/image.h"
#include "gv/model/quad.h"
#include "gv/model/tag_lookup.h"

namespace gv::model {

using Homography = std::array<double, 9>;  // row-major, maps (u, v, 1) in the unit square

Homography square_to_quad(const std::array<std::array<double, 2>, 4>& quad);
std::array<double, 2> project(const Homography& h, double u, double v);
// Tag coordinates (-1..1) to image coordinates.
std::array<double, 2> project_tag(const Homography& h, double tag_x, double tag_y);

struct CodeRead {
  uint64_t code = 0;          // 36 bits, MSB first in the family's cell order
  double decision_margin = 0;
};

// nullopt if the border is reversed (inside brighter than outside) or the
// quad leaves the image.
std::optional<CodeRead> read_code(const Image8& image, const std::array<std::array<double, 2>, 4>& quad,
                                  const Tag36h11Lookup& family, double sharpening = 0.25);

}  // namespace gv::model
