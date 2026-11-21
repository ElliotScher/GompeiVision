// Checks the scene generator itself: AprilTag must find the tags it drew, with
// corners near where it drew them. Otherwise the other tests prove nothing.
#include <gtest/gtest.h>

#include <cmath>

#include "apriltag.h"
#include "apriltag_ref.h"
#include "synth.h"
#include "tag36h11.h"

TEST(Synth, AprilTagDetectsGeneratedTags) {
  apriltag_family_t* fam = tag36h11_create();
  apriltag_detector_t* td = apriltag_detector_create();
  apriltag_detector_add_family(td, fam);
  td->quad_decimate = 1;
  td->nthreads = 1;
  int drawn = 0, found = 0;
  for (uint32_t seed = 1; seed <= 20; ++seed) {
    gv::test::SceneParams p;
    p.num_tags = 2, p.min_tag_px = 70, p.max_tag_px = 140, p.max_skew = 0.15;
    const auto scene = gv::test::make_scene(seed, p);
    image_u8_t v = gv::test::view(scene.image);
    zarray_t* dets = apriltag_detector_detect(td, &v);
    for (const auto& t : scene.tags) {
      ++drawn;
      for (int i = 0; i < zarray_size(dets); ++i) {
        apriltag_detection_t* d;
        zarray_get(dets, i, &d);
        if (d->id != t.id) continue;
        // Corner order conventions differ, so match each detected corner to
        // the nearest drawn one.
        double err = 0;
        for (int k = 0; k < 4; ++k) {
          double best = 1e9;
          for (int j = 0; j < 4; ++j)
            best = std::min(best, std::hypot(d->p[k][0] - t.corners[j][0], d->p[k][1] - t.corners[j][1]));
          err = std::max(err, best);
        }
        EXPECT_LT(err, 2.0) << "seed " << seed << " tag " << t.id;
        ++found;
        break;
      }
    }
    apriltag_detections_destroy(dets);
  }
  apriltag_detector_destroy(td);
  tag36h11_destroy(fam);
  // Tags can overlap or leave the frame; most must be found.
  EXPECT_GE(found * 10, drawn * 9) << found << " of " << drawn << " generated tags detected";
}
