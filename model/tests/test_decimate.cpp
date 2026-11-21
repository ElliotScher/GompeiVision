#include <gtest/gtest.h>

#include "apriltag_ref.h"
#include "gv/model/decimate.h"
#include "synth.h"

using gv::model::decimate;

TEST(Decimate, MatchesAprilTagForIntegerFactors) {
  for (int factor : {2, 3, 4}) {
    for (auto [w, h] : {std::pair{640, 400}, {1280, 800}, {33, 17}, {5, 9}, {1, 1}}) {
      const auto img = gv::test::noise_image(static_cast<uint32_t>(factor * 1000 + w + h), w, h);
      EXPECT_EQ(decimate(img, factor), gv::test::ref_decimate(img, static_cast<float>(factor)))
          << "factor " << factor << ", " << w << "x" << h;
    }
  }
}

TEST(Decimate, FactorOneIsIdentity) {
  const auto img = gv::test::noise_image(7, 20, 10);
  EXPECT_EQ(decimate(img, 1), img);
}
