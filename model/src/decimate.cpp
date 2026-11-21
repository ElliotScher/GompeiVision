#include "gv/model/decimate.h"

#include <stdexcept>

namespace gv::model {

Image8 decimate(const Image8& in, int factor) {
  if (factor < 1) throw std::invalid_argument("decimate: factor must be >= 1");
  if (factor == 1) return in;
  Image8 out(1 + (in.width - 1) / factor, 1 + (in.height - 1) / factor);
  for (int sy = 0; sy < out.height; ++sy)
    for (int sx = 0; sx < out.width; ++sx) out.at(sx, sy) = in.at(sx * factor, sy * factor);
  return out;
}

}  // namespace gv::model
