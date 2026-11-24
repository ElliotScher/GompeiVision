// Fixed-point emulation for the Stage 4 error budget. A Quantization rounds
// the quantities the PL would carry, at the points it would round them, so
// the golden model can measure how few bits keep the answers. Every field 0
// (the default) means "double precision", and the model is bit-identical to
// running without it.
#pragma once

#include <cmath>

namespace gv::model {

struct Quantization {
  int coord_frac = 0;       // fractional bits of pixel coordinates (corners, sample points, edge points)
  int weight_frac = 0;      // fractional bits of bilinear interpolation weights
  int direction_frac = 0;   // fractional bits of an edge's unit direction and normal
  int value_frac = 0;       // fractional bits of interpolated pixel values
  int homography_bits = 0;  // significant bits per homography coefficient (they span ~1e-3..1e3)
  // Fractional bits of the brightness-plane coefficients (decode), fitted in
  // tag coordinates scaled by 8, which are integers: the fit's sums and
  // determinants are exact, so only the coefficients are rounded. With it
  // set, the decision margin is also rounded to value_frac bits.
  int plane_frac = 0;
  // A PL projector instead (decode): when both are set, the homography is
  // fixed point, h0..h5 (pixel units) with proj_scale_frac fractional bits and
  // the perspective terms h6, h7 with proj_persp_frac (h8 = 1), and each point
  // is one exact integer division, rounded to coord_frac bits. Replaces
  // homography_bits for decode.
  int proj_scale_frac = 0;
  int proj_persp_frac = 0;
  bool fixed_projector() const { return proj_scale_frac > 0 && proj_persp_frac > 0; }
  bool enabled() const {
    return coord_frac || weight_frac || direction_frac || value_frac || homography_bits || plane_frac ||
           fixed_projector();
  }
};

// The widths chosen for Stage 4's PL refinement and decode (2026-11-22, from
// tests/test_fixed_point.cpp): they add a median 0.004 px and at most about
// 0.04 px of corner error to double precision, with no tag gained or lost.
// The golden model run with these widths is the RTL's bit-exact reference.
// Plane coefficients: 8 fractional bits. On synthetic tags even 2 changed
// nothing (their decision margins are large); 8 leaves room for low-contrast
// tags, to re-check on recordings.
// Decode's projection runs in the PL (FixedProjector): homography scale terms
// with 14 fractional bits, perspective terms 24, in 32-bit registers. Every
// sample point is within 1/256 px of a double-precision projection (99% of
// them equal), with 8-16x headroom over the largest coefficients seen.
// (homography_bits, the floating-point-like alternative, is off: at 14 bits
// it moved points up to 0.4 px.)
inline constexpr Quantization kStage4Widths{8, 8, 12, 4, 0, 8, 14, 24};

// Round to `frac` fractional bits (no-op for frac <= 0).
inline double quantize(double v, int frac) {
  if (frac <= 0) return v;
  const double s = std::ldexp(1.0, frac);
  return std::round(v * s) / s;
}

// Round to `bits` significant bits (no-op for bits <= 0).
inline double quantize_relative(double v, int bits) {
  if (bits <= 0 || v == 0 || !std::isfinite(v)) return v;
  const int e = std::ilogb(v);
  const double s = std::ldexp(1.0, bits - 1 - e);
  return std::round(v * s) / s;
}

}  // namespace gv::model
