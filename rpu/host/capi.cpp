// C interface to the finisher, so the RTL testbenches (Python) run the RPU's
// code against the simulated PL (sim/detector).
#include "gv/rpu/finisher.h"

using gv::rpu::Finisher;

extern "C" {

Finisher* gv_rpu_new(int width, int height) {
  gv::rpu::Config c;
  c.width = width, c.height = height;
  return new Finisher(c);
}
void gv_rpu_delete(Finisher* f) { delete f; }
void gv_rpu_quad(Finisher* f, uint32_t frame, const int32_t* cx, const int32_t* cy) { f->on_quad(frame, cx, cy); }
void gv_rpu_frame_done(Finisher* f, uint32_t frame, int aborted) { f->on_frame_done(frame, aborted != 0); }
void gv_rpu_read(Finisher* f, uint32_t tag, int ok, int32_t margin, int found, int32_t id, int32_t rot, int32_t ham) {
  f->on_read(tag, ok != 0, margin, found != 0, id, rot, ham);
}
void gv_rpu_point(Finisher* f, uint32_t tag, int32_t x, int32_t y) { f->on_point(tag, x, y); }
void gv_rpu_edge_done(Finisher* f, uint32_t tag) { f->on_edge_done(tag); }
void gv_rpu_progress(Finisher* f, uint32_t frame, int32_t rows) { f->on_progress(frame, rows); }
// Jobs as int32 arrays: {tag, h0..h7}, and {tag, a_x .. check_direction}.
int gv_rpu_next_read(Finisher* f, int32_t* out) {
  gv::rpu::ReadJob j;
  if (!f->next_read(j)) return 0;
  out[0] = static_cast<int32_t>(j.tag);
  for (int i = 0; i < 8; ++i) out[1 + i] = j.h[i];
  return 1;
}
int gv_rpu_next_edge(Finisher* f, int32_t* out) {
  gv::rpu::EdgeJob j;
  if (!f->next_edge(j)) return 0;
  const int32_t v[13] = {static_cast<int32_t>(j.tag), j.a_x, j.a_y, j.d_x, j.d_y, j.n_x, j.n_y, j.t0, j.spacing,
                         j.count, j.half_steps, j.window_steps, j.check_direction};
  for (int i = 0; i < 13; ++i) out[i] = v[i];
  return 1;
}
// {frame, aborted, count}, then per tag {id, hamming, margin, cut, x0, y0 .. x3, y3}.
int gv_rpu_pop_frame(Finisher* f, int32_t* out) {
  static gv::rpu::FrameResult r;
  if (!f->pop_frame(r)) return 0;
  *out++ = static_cast<int32_t>(r.frame), *out++ = r.aborted, *out++ = r.count;
  for (int i = 0; i < r.count; ++i) {
    const auto& t = r.tags[i];
    *out++ = t.id, *out++ = t.hamming, *out++ = t.margin, *out++ = t.cut_by_frame;
    for (const auto& c : t.corners) *out++ = c[0], *out++ = c[1];
  }
  return 1;
}
void gv_rpu_counters(Finisher* f, uint32_t* out) {
  const auto& c = f->counters();
  const uint32_t v[10] = {c.quads, c.dropped_full, c.dropped_stale, c.dropped_detections, c.points_lost,
                          c.bad_projector, c.quick_fail, c.refine_fail, c.decode_fail, c.duplicates};
  for (int i = 0; i < 10; ++i) out[i] = v[i];
}

}  // extern "C"
