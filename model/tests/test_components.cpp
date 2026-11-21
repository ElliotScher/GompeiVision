#include <gtest/gtest.h>

#include <map>
#include <set>

#include "apriltag_ref.h"
#include "gv/model/components.h"
#include "gv/model/decimate.h"
#include "gv/model/threshold.h"
#include "synth.h"

using namespace gv::model;

namespace {

// AprilTag's threaded connected_components() never links the last row when
// (h - 1) is a multiple of its chunk size: the last task stops one row short,
// and no later task stitches that row. The single-threaded result is the
// correct one, and the model's definition. The threaded result only matches
// it when this is false.
bool apriltag_threads_skip_last_row(int h, int nthreads) {
  if (nthreads <= 1) return false;
  const int chunk = 1 + h / (APRILTAG_TASKS_PER_THREAD_TARGET * nthreads);
  return (h - 1) % chunk == 0;
}

void expect_partition_matches(const Image8& t, const std::string& what) {
  const Labeling l = label_components(t);
  for (int nthreads : {1, 2}) {  // the CPU pipeline runs AprilTag with 2 threads
    if (apriltag_threads_skip_last_row(t.height, nthreads)) continue;
    const auto ref = gv::test::ref_components(t, nthreads);
    size_t diffs = 0, first = 0;
    for (size_t i = 0; i < ref.size(); ++i)
      if (l.label[i] != ref[i] && diffs++ == 0) first = i;
    EXPECT_EQ(diffs, 0u) << what << " (nthreads " << nthreads << "): first mismatch at x="
                         << first % static_cast<size_t>(t.width) << " y=" << first / static_cast<size_t>(t.width);
  }
}

// Brute-force accumulators from the final labels, to check the streamed ones.
std::map<uint32_t, Component> accumulate(const Image8& t, const Labeling& l) {
  std::map<uint32_t, Component> m;
  for (int y = 0; y < t.height; ++y)
    for (int x = 0; x < t.width; ++x) {
      if (t.at(x, y) == kSkip) continue;
      const uint32_t id = l.label[t.index(x, y)];
      auto [it, fresh] = m.try_emplace(id, Component{id, t.at(x, y), 0, x, x, y, y, 0, {}});
      Component& c = it->second;
      c.area++;
      c.x_min = std::min(c.x_min, x), c.x_max = std::max(c.x_max, x);
      c.y_min = std::min(c.y_min, y), c.y_max = std::max(c.y_max, y);
      // Extremes by definition: the largest score, first in raster order.
      for (size_t k = 0; k < kDirections; ++k) {
        const int32_t s = kDirectionVectors[k][0] * x + kDirectionVectors[k][1] * y;
        if (fresh || s > c.extremes[k].score) c.extremes[k] = {x, y, s};
      }
    }
  for (auto& [id, c] : m) c.closed_after_row = c.y_max + 1 < t.height ? c.y_max + 1 : t.height - 1;
  return m;
}

void expect_streamed_accumulators_correct(const Image8& t, const std::string& what) {
  const Labeling l = label_components(t);
  const auto truth = accumulate(t, l);
  std::set<uint32_t> seen;
  for (const Component& c : l.closed) {
    EXPECT_TRUE(seen.insert(c.id).second) << what << ": component " << c.id << " closed twice";
    auto it = truth.find(c.id);
    ASSERT_NE(it, truth.end()) << what << ": closed unknown component " << c.id;
    EXPECT_EQ(c, it->second) << what << ": component " << c.id;
  }
  EXPECT_EQ(seen.size(), truth.size()) << what << ": some components never closed";
  // Closure order follows the rows.
  for (size_t i = 1; i < l.closed.size(); ++i)
    EXPECT_LE(l.closed[i - 1].closed_after_row, l.closed[i].closed_after_row) << what;
}

}  // namespace

TEST(Components, MatchesAprilTagOnTagScenes) {
  for (uint32_t seed = 1; seed <= 30; ++seed) {
    const auto scene = gv::test::make_scene(seed);
    expect_partition_matches(threshold(scene.image).image, "scene " + std::to_string(seed));
  }
}

TEST(Components, MatchesAprilTagOnDecimatedFullResScenes) {
  gv::test::SceneParams p;
  p.width = 1280, p.height = 800, p.num_tags = 6, p.min_tag_px = 60, p.max_tag_px = 400;
  for (uint32_t seed = 200; seed < 204; ++seed) {
    const auto scene = gv::test::make_scene(seed, p);
    expect_partition_matches(threshold(decimate(scene.image, 4)).image, "decimated " + std::to_string(seed));
    expect_partition_matches(threshold(scene.image).image, "full-res " + std::to_string(seed));
  }
}

TEST(Components, MatchesAprilTagOnAdversarialImages) {
  // Pure noise thresholds into a checkerboard-like mess: the worst case for
  // the edge-column join rules and the equivalence table.
  for (auto [w, h] : {std::pair{4, 4}, {5, 6}, {17, 9}, {64, 48}, {160, 100}}) {
    for (uint32_t seed = 0; seed < 6; ++seed) {
      const auto noise = gv::test::noise_image(seed * 97 + static_cast<uint32_t>(w), w, h);
      expect_partition_matches(threshold(noise).image, "noise " + std::to_string(w) + "x" + std::to_string(h));
      const auto blocks = gv::test::blocks_image(seed * 89 + static_cast<uint32_t>(h), w, h, 25);
      expect_partition_matches(threshold(blocks).image, "blocks " + std::to_string(w) + "x" + std::to_string(h));
    }
  }
}

TEST(Components, StreamedAccumulatorsAndClosuresAreCorrect) {
  for (uint32_t seed = 1; seed <= 10; ++seed) {
    expect_streamed_accumulators_correct(threshold(gv::test::make_scene(seed).image).image,
                                         "scene " + std::to_string(seed));
    expect_streamed_accumulators_correct(threshold(gv::test::noise_image(seed, 64, 48)).image,
                                         "noise " + std::to_string(seed));
  }
}

TEST(Components, LastColumnOnlyJoinsDiagonallyFromBelow) {
  // The reference never joins (w-1, y) to (w-2, y) directly. A white
  // column at x = w-1 next to a white column at x = w-2 stays separate
  // per pixel unless a white pixel below at (w-2, y+1) links it (and the
  // up/up-right skip doesn't fire).
  Image8 t(6, 3, 0);
  for (int y = 0; y < 3; ++y) t.at(5, y) = 255, t.at(4, y) = 255;
  expect_partition_matches(t, "edge column");
}

TEST(Components, AprilTagThreadedLastRowQuirkIsAsPredicted) {
  // Guards the skip in expect_partition_matches: where the quirk is
  // predicted, threaded AprilTag really does differ from single-threaded
  // (on an image busy enough to show it), and only in the last row's links.
  for (int h : {4, 6, 9, 400}) {
    ASSERT_TRUE(apriltag_threads_skip_last_row(h, 2)) << h;
    const Image8 t = threshold(gv::test::noise_image(static_cast<uint32_t>(h), 64, h)).image;
    EXPECT_NE(gv::test::ref_components(t, 2), gv::test::ref_components(t, 1)) << "h=" << h;
    // Blanking the last row (all 127) removes every link it would make, and
    // then the two agree.
    Image8 cut = t;
    for (int x = 0; x < cut.width; ++x) cut.at(x, h - 1) = kSkip;
    EXPECT_EQ(gv::test::ref_components(cut, 2), gv::test::ref_components(cut, 1)) << "h=" << h;
  }
}

TEST(Components, CpuPipelineConfigIsUnaffectedByThreadQuirk) {
  // 1280x800 with quad_decimate = 4 thresholds a 320x200 image; with
  // nthreads = 2 AprilTag's threaded result is the single-threaded one.
  EXPECT_FALSE(apriltag_threads_skip_last_row(200, 2));
  EXPECT_FALSE(apriltag_threads_skip_last_row(800, 2));
  gv::test::SceneParams p;
  p.width = 1280, p.height = 800, p.num_tags = 6, p.min_tag_px = 60, p.max_tag_px = 400;
  const Image8 t = threshold(decimate(gv::test::make_scene(300, p).image, 4)).image;
  EXPECT_EQ(gv::test::ref_components(t, 2), gv::test::ref_components(t, 1));
}
