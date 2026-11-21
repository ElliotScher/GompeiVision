#include <gtest/gtest.h>

#include "apriltag_ref.h"
#include "gv/model/decimate.h"
#include "gv/model/threshold.h"
#include "synth.h"

using namespace gv::model;

namespace {

void expect_matches_reference(const Image8& img, int min_diff, const std::string& what) {
  const ThresholdResult r = threshold(img, {min_diff});
  const Image8 ref = gv::test::ref_threshold(img, min_diff);
  ASSERT_EQ(r.image.width, ref.width);
  size_t diffs = 0, first = 0;
  for (size_t i = 0; i < ref.pixels.size(); ++i)
    if (r.image.pixels[i] != ref.pixels[i] && diffs++ == 0) first = i;
  EXPECT_EQ(diffs, 0u) << what << ": first mismatch at x=" << first % static_cast<size_t>(ref.width)
                       << " y=" << first / static_cast<size_t>(ref.width);
}

}  // namespace

TEST(Threshold, MatchesAprilTagOnTagScenes) {
  for (uint32_t seed = 1; seed <= 40; ++seed) {
    const auto scene = gv::test::make_scene(seed);
    expect_matches_reference(scene.image, 5, "scene " + std::to_string(seed));
  }
}

TEST(Threshold, MatchesAprilTagOnDecimatedFullResScenes) {
  // The CPU pipeline's configuration: 1280x800, quad_decimate = 4.
  gv::test::SceneParams p;
  p.width = 1280, p.height = 800, p.num_tags = 6, p.min_tag_px = 60, p.max_tag_px = 400;
  for (uint32_t seed = 100; seed < 105; ++seed) {
    const auto scene = gv::test::make_scene(seed, p);
    expect_matches_reference(decimate(scene.image, 4), 5, "decimated scene " + std::to_string(seed));
  }
}

TEST(Threshold, MatchesAprilTagOnOddSizesAndEdgeTiles) {
  // Sizes that aren't multiples of 4 exercise the partial-tile quirk.
  for (auto [w, h] : {std::pair{4, 4}, {5, 4}, {4, 7}, {7, 7}, {33, 17}, {321, 201}, {320, 200}}) {
    const auto blocks = gv::test::blocks_image(static_cast<uint32_t>(w * 31 + h), w, h);
    expect_matches_reference(blocks, 5, "blocks " + std::to_string(w) + "x" + std::to_string(h));
    const auto noise = gv::test::noise_image(static_cast<uint32_t>(w * 17 + h), w, h, 100, 108);
    expect_matches_reference(noise, 5, "low-contrast noise " + std::to_string(w) + "x" + std::to_string(h));
  }
}

TEST(Threshold, MatchesAprilTagAcrossMinWhiteBlackDiff) {
  const auto scene = gv::test::make_scene(77);
  for (int d : {0, 1, 5, 20, 60, 255})
    expect_matches_reference(scene.image, d, "min_white_black_diff " + std::to_string(d));
}

TEST(Threshold, PartialEdgeTilesAreNeverSkipped) {
  // A flat image is all low-contrast: every full tile is 127, but the
  // partial right column and bottom row are still binarized (to 0).
  const Image8 flat(10, 9, 100);
  const Image8 t = threshold(flat).image;
  for (int y = 0; y < 9; ++y)
    for (int x = 0; x < 10; ++x) EXPECT_EQ(t.at(x, y), (x < 8 && y < 8) ? kSkip : 0) << x << "," << y;
}

TEST(Threshold, RejectsImagesSmallerThanOneTile) {
  EXPECT_THROW(threshold(Image8(3, 8)), std::invalid_argument);
}
