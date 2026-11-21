// Synthetic test scenes: real tag36h11 codes drawn with perspective, blur,
// noise and lighting gradients, plus pure-noise and blocky images. Deterministic
// for a given seed. Used until recorded datasets exist (Stage 1), and kept
// afterwards for corner cases recordings rarely hit.
#pragma once

#include <cstdint>
#include <random>
#include <vector>

#include "gv/model/image.h"

namespace gv::test {

struct PlacedTag {
  int id = 0;
  double corners[4][2] = {};  // outer black border, TL TR BR BL in image pixels
};

struct Scene {
  model::Image8 image;
  std::vector<PlacedTag> tags;
};

struct SceneParams {
  int width = 640;
  int height = 400;
  int num_tags = 3;
  double min_tag_px = 40;   // black-border width range in pixels
  double max_tag_px = 160;
  double max_skew = 0.25;   // corner jitter as a fraction of tag size (perspective)
  int noise = 6;            // +/- uniform noise amplitude
  bool blur = true;         // 3x3 box blur
  int extra_blur = 0;       // further 3x3 box-blur passes (motion/defocus stress)
  double background_min = 60, background_max = 190;  // background brightness range
};

Scene make_scene(uint32_t seed, const SceneParams& params = {});

// Uniform random pixels in [lo, hi].
model::Image8 noise_image(uint32_t seed, int w, int h, int lo = 0, int hi = 255);

// Random rectangles of random gray levels: large flat regions and sharp edges,
// which stress the low-contrast (127) and component-merging paths.
model::Image8 blocks_image(uint32_t seed, int w, int h, int num_blocks = 40);

}  // namespace gv::test
