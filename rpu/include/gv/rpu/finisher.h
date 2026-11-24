// The RPU's part of the tag detector (Stage 4): rough quads in from the PL's
// quad finder, finished tags out. It sets up the PL's work and does the few
// steps that need a square root or a division:
//   quad -> quick read (grown rough quad's homography; PL decode)
//        -> four edge jobs (PL edge sampler) -> line fits -> corners
//        -> final read (refined corners' homography; PL decode) -> tag
// The same steps, in double precision, as the golden detector
// (model/src/detector.cpp, refine.cpp) at the Stage 4 widths.
//
// Event driven: feed PL results to the on_*() functions, and take jobs with
// next_read() and next_edge() whenever the PL can accept one. No heap, no
// exceptions; one instance is a fixed-size object.
//
// Rows. The quad arrives a few rows after the tag's last one, but a read
// samples half a cell outside the tag (25 px for a 400 px tag) and refinement
// up to 7 px, so a job may need rows that aren't in yet. on_progress() says
// how many of the current frame's rows are in, and a job is only released when
// its rows are. The pixel store is a ring: rows go when it wraps (if it holds
// less than a frame) and when the next frame overwrites them from the top. A
// candidate whose rows are gone, or about to go, is dropped and counted.
#pragma once

#include <cstdint>

namespace gv::rpu {

inline constexpr int kMaxCandidates = 64;  // quads being finished at once
inline constexpr int kMaxFrames = 4;       // frames with unfinished business
inline constexpr int kMaxDetections = 64;  // per frame
inline constexpr int kMaxPoints = 1024;    // per edge: 1280x800's diagonal at 1.5 px is 1,007

struct Config {
  int width = 1280, height = 800;
  int rows_kept = 1024;                        // the PL's pixel store (pixel_window's DEPTH)
  double quick_grow = 1.0;                     // px, the quick read's quad grows by this
  double window_min = 2.0, window_max = 6.0;   // px, the edge search
  double outlier_px = 1.0;                     // edge points farther than this from the first fit
  int min_points = 4;                          // per edge, after outlier rejection
  // Rows a job may still be reading while the writer moves on: the longest
  // edge job is about 27,000 cycles, or 21 rows at 1,300 cycles each.
  int stale_margin_rows = 32;
};

// PL job registers. `tag` comes back with the result.
struct ReadJob {
  uint32_t tag;
  int32_t h[8];  // h0..h5 Q.14, h6, h7 Q.24
};
struct EdgeJob {
  uint32_t tag;
  int32_t a_x, a_y;            // Q.8
  int32_t d_x, d_y, n_x, n_y;  // Q.12
  int32_t t0, spacing;         // Q.8
  int32_t count, half_steps, window_steps, check_direction;
};

struct Detection {
  int32_t id, hamming;
  int32_t margin;             // Q.4
  int32_t corners[4][2];      // Q.8, TL TR BR BL of the tag as printed
  bool cut_by_frame;          // a corner is outside the image (extrapolated)
  uint32_t seq;               // order of arrival of its quad
};

struct Counters {  // since construction
  uint32_t quads, dropped_full, dropped_stale, dropped_detections, points_lost;
  uint32_t bad_projector, quick_fail, refine_fail, decode_fail, duplicates;
};

struct FrameResult {
  uint32_t frame;
  bool aborted;  // the quad finder cut the frame short
  int count;
  Detection tags[kMaxDetections];
};

class Finisher {
 public:
  explicit Finisher(const Config& config);

  // PL events. A quad's corners are pixel indices, in order around it.
  void on_quad(uint32_t frame, const int32_t cx[4], const int32_t cy[4]);
  void on_frame_done(uint32_t frame, bool aborted);
  void on_read(uint32_t tag, bool ok, int32_t margin, bool found, int32_t id, int32_t rotation, int32_t hamming);
  void on_point(uint32_t tag, int32_t x, int32_t y);  // Q.8; an edge job's points come together, in order
  void on_edge_done(uint32_t tag);
  void on_progress(uint32_t frame, int32_t rows);  // `rows` whole rows of `frame` are in

  // Jobs to send. Each true result must reach the PL.
  bool next_read(ReadJob& job);
  bool next_edge(EdgeJob& job);

  // Finished frames, oldest first, when every one of their quads is done.
  bool pop_frame(FrameResult& out);

  const Counters& counters() const { return counters_; }

 private:
  enum class State : uint8_t { kFree, kQuickReady, kQuickBusy, kEdgesReady, kEdgesBusy, kFinalReady, kFinalBusy };
  struct Line {
    double px, py, dx, dy;  // a point and the unit direction
    bool valid;
  };
  struct Candidate {
    State state;
    uint8_t frame_slot;
    uint8_t edges_done;
    uint32_t seq;
    double rough[4][2];
    int32_t refined[4][2];      // Q.8
    int32_t row_min, row_max;   // image rows the next job reads
    ReadJob read;
    EdgeJob edges[4];
    Line lines[4];
  };
  struct Frame {
    bool used, marked, aborted;
    uint32_t id, order;
    int outstanding;
    int count;
    Detection tags[kMaxDetections];
  };

  enum class Rows : uint8_t { kReady, kWait, kGone };
  Rows rows_of(const Candidate& c) const;
  int frame_slot(uint32_t frame, bool create);
  bool setup_read(Candidate& c, const double corners[4][2]);
  void setup_edges(Candidate& c);
  bool fit(Line& line, int n) const;  // the first n points of points_
  void finish(Candidate& c);          // its outcome is known
  void free_stale();
  uint32_t tag_of(const Candidate& c, int which) const;

  Config config_;
  Counters counters_{};
  Candidate candidates_[kMaxCandidates];
  Frame frames_[kMaxFrames];
  uint32_t next_seq_ = 0, next_order_ = 0;
  uint32_t progress_frame_ = 0;
  int32_t progress_rows_ = -1;  // -1: nothing known yet
  int sending_ = -1;            // candidate whose edge jobs are going out
  int send_edge_ = 0;
  int32_t points_[kMaxPoints][2];
  int num_points_ = 0;
};

}  // namespace gv::rpu
