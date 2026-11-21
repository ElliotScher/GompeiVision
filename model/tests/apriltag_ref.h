// Calls into the AprilTag library's own threshold and connected-components
// code, which the golden models must match bit for bit. These functions are
// internal to the library (no public header) but have external linkage in the
// static build.
#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "apriltag.h"
#include "common/image_u8.h"
#include "common/unionfind.h"
#include "common/workerpool.h"
#include "gv/model/detector.h"
#include "gv/model/image.h"
#include "tag36h11.h"

extern "C" {
image_u8_t* threshold(apriltag_detector_t* td, image_u8_t* im);
unionfind_t* connected_components(apriltag_detector_t* td, image_u8_t* threshim, int w, int h, int ts);
}

namespace gv::test {

struct DetectorDeleter {
  void operator()(apriltag_detector_t* td) const { apriltag_detector_destroy(td); }
};
using Detector = std::unique_ptr<apriltag_detector_t, DetectorDeleter>;

inline Detector make_detector(int nthreads = 1, int min_white_black_diff = 5) {
  Detector td(apriltag_detector_create());
  td->nthreads = nthreads;
  td->qtp.min_white_black_diff = min_white_black_diff;
  // connected_components() uses td->wp directly; detect() normally makes it.
  td->wp = workerpool_create(nthreads);
  return td;
}

// A borrowed image_u8_t view (stride == width) over a model image.
inline image_u8_t view(const model::Image8& img) {
  return image_u8_t{img.width, img.height, img.width, const_cast<uint8_t*>(img.pixels.data())};
}

inline model::Image8 to_model(const image_u8_t* im) {
  model::Image8 out(im->width, im->height);
  for (int y = 0; y < im->height; ++y)
    for (int x = 0; x < im->width; ++x) out.at(x, y) = im->buf[y * im->stride + x];
  return out;
}

inline model::Image8 ref_decimate(const model::Image8& in, float factor) {
  image_u8_t v = view(in);
  image_u8_t* d = image_u8_decimate(&v, factor);
  model::Image8 out = to_model(d);
  image_u8_destroy(d);
  return out;
}

inline model::Image8 ref_threshold(const model::Image8& in, int min_white_black_diff = 5) {
  Detector td = make_detector(1, min_white_black_diff);
  image_u8_t v = view(in);
  image_u8_t* t = threshold(td.get(), &v);
  model::Image8 out = to_model(t);
  image_u8_destroy(t);
  return out;
}

// Per pixel: the smallest raster index in its union-find set, the same
// canonical form as model::Labeling::label.
inline std::vector<uint32_t> ref_components(const model::Image8& thresholded, int nthreads = 1) {
  Detector td = make_detector(nthreads);
  image_u8_t v = view(thresholded);
  const int w = thresholded.width, h = thresholded.height;
  unionfind_t* uf = connected_components(td.get(), &v, w, h, w);
  const auto n = static_cast<uint32_t>(w * h);
  std::vector<uint32_t> first(n, UINT32_MAX), label(n);
  for (uint32_t i = 0; i < n; ++i) {
    const uint32_t r = unionfind_get_representative(uf, i);
    if (first[r] == UINT32_MAX) first[r] = i;
    label[i] = first[r];
  }
  unionfind_destroy(uf);
  return label;
}

// AprilTag as the CPU pipeline runs it (refine_edges, default sharpening),
// with corners remapped to TL, TR, BR, BL exactly as FiducialDetector does.
inline std::vector<model::Detection> apriltag_detect(const model::Image8& img, float decimate) {
  static apriltag_family_t* fam = tag36h11_create();
  apriltag_detector_t* td = apriltag_detector_create();
  apriltag_detector_add_family(td, fam);
  td->quad_decimate = decimate;
  td->nthreads = 1;
  td->refine_edges = true;
  image_u8_t v = view(img);
  zarray_t* dets = apriltag_detector_detect(td, &v);
  std::vector<model::Detection> out;
  for (int i = 0; i < zarray_size(dets); ++i) {
    apriltag_detection_t* d;
    zarray_get(dets, i, &d);
    model::Detection x;
    x.id = d->id, x.hamming = d->hamming, x.decision_margin = d->decision_margin;
    const int order[4] = {3, 2, 1, 0};
    for (size_t k = 0; k < 4; ++k) x.corners[k] = {d->p[order[k]][0], d->p[order[k]][1]};
    out.push_back(x);
  }
  apriltag_detections_destroy(dets);
  apriltag_detector_destroy(td);
  return out;
}

}  // namespace gv::test
