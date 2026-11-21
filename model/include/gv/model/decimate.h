// Optional decimation before thresholding (AprilTag's quad_decimate).
//
// Bit-exact with image_u8_decimate() in AprilTag v3.4.5 for integer factors:
// output pixel (sx, sy) is input pixel (sx * f, sy * f), so the output is
// 1 + (w - 1) / f by 1 + (h - 1) / f. No averaging. The CPU pipeline runs with
// f = 4. AprilTag's special 1.5 filter is not modeled.
#pragma once

#include "gv/model/image.h"

namespace gv::model {

Image8 decimate(const Image8& in, int factor);

}  // namespace gv::model
