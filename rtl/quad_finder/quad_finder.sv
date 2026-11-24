// Pixels in, rough quads out: the detector's front end (PLAN steps 2-4B), first
// version. Threshold -> pixel FIFO -> connected components -> candidate filter
// -> record FIFO -> corner scorecard.
//
// Quads come out in the golden model's order (component closure order), each
// with its frame ID. After a frame's last quad, o_frame_done reports the frame:
// its counters, and whether it was cut short.
//
// Nothing upstream of the sensor can wait, so overload drops and counts:
//   - pixel FIFO full (components falls behind the scan): the rest of the frame
//     is dropped and the block resumes at the next start of frame. Quads
//     already closed still come out, and the frame's o_frame_done has
//     o_fd_aborted set. It comes when the next frame reaches components. A
//     frame with no room for its first pixel is dropped whole and gets no
//     o_frame_done; o_frames_cut counts both kinds.
//   - record FIFO full (the scorecard falls behind): the candidate is dropped
//     and counted in o_fd_dropped. The FIFO keeps MarkSlots entries free for
//     end-of-frame markers.
//
// Candidate filter, none of which can change which quads are accepted: black
// (the golden detector's rule), area >= MIN_AREA (likewise), and a bounding box
// whose diagonal is at least MIN_SIDE (any shorter and every side of the quad
// is too, so the scorecard would reject it).
module quad_finder #(
    parameter int MAX_W = 1280,
    parameter int MAX_LABELS = 4096,
    parameter int PIX_DEPTH = 16384,
    parameter int REC_DEPTH = 128,
    parameter int MIN_AREA = 16,
    parameter int MIN_SIDE = 8
) (
    input logic        clk,
    input logic        rst,
    input logic [ 7:0] min_white_black_diff,
    // Pixels, as rtl/threshold takes them. i_sof latches the frame ID and size.
    input logic [31:0] i_frame,
    input logic [11:0] i_width,
    input logic [11:0] i_height,
    input logic        i_valid,
    input logic        i_sof,
    input logic [ 7:0] i_pix,

    output logic               o_valid,              // one accepted quad
    output logic        [31:0] o_frame,
    output logic        [31:0] o_id,
    output logic        [11:0] o_cx            [4],
    output logic        [11:0] o_cy            [4],
    output logic signed [31:0] o_area2,
    output logic               o_frame_done,
    output logic        [31:0] o_fd_frame,
    output logic               o_fd_aborted,
    output logic        [31:0] o_fd_candidates,      // black, area >= MIN_AREA
    output logic        [31:0] o_fd_small,           // of those, failed the diagonal filter
    output logic        [31:0] o_fd_dropped,         // of those, lost to a full record FIFO
    output logic        [31:0] o_fd_quads,           // accepted by the scorecard
    output logic        [31:0] o_fd_off_quad,        // rejected as extremes off quad (4C's input)
    output logic        [31:0] o_frames_cut,
    output logic        [31:0] o_pix_level_max,
    output logic        [31:0] o_rec_level_max,
    output logic        [31:0] o_row_cycles_max,
    output logic        [31:0] o_labels_max,
    output logic               o_error               // sticky: overrun or label/marker overflow
);

  localparam int PixA = $clog2(PIX_DEPTH);
  localparam int RecA = $clog2(REC_DEPTH);
  localparam int MarkSlots = 4;

  // ------------------------------------------------------------ threshold --
  logic t_valid, t_sof, t_overrun;
  logic [7:0] t_pix;
  /* verilator lint_off PINCONNECTEMPTY */
  threshold #(
      .MAX_W(MAX_W)
  ) u_threshold (
      .clk,
      .rst,
      .min_white_black_diff,
      .i_width,
      .i_height,
      .i_valid,
      .i_sof,
      .i_pix,
      .o_valid(t_valid),
      .o_sof(t_sof),
      .o_eol(),
      .o_x(),
      .o_y(),
      .o_pix(t_pix),
      .o_overrun(t_overrun)
  );
  /* verilator lint_on PINCONNECTEMPTY */

  // A frame finishes leaving the threshold before the next one enters (or
  // t_overrun is set), so one set of latched frame data is enough here.
  typedef struct packed {
    logic [31:0] frame;
    logic [11:0] width;
    logic [11:0] height;
  } info_t;
  info_t in_info;
  always_ff @(posedge clk)
    if (i_valid && i_sof)
      in_info <= '{frame: i_frame, width: i_width, height: i_height};

  // ----------------------------------------------------------- pixel FIFO --
  logic [2:0] pix_mem[PIX_DEPTH];  // {sof, pixel code}
  info_t info_mem[4];
  logic [PixA:0] pix_w, pix_r;
  logic [2:0] info_w, info_r, info_o;  // write, components' input, components' output
  logic keeping;  // the frame leaving the threshold is being kept
  wire [PixA:0] pix_level = pix_w - pix_r;
  wire pix_full = pix_level == (PixA + 1)'(PIX_DEPTH);
  wire info_full = (info_w - info_o) == 3'd4;
  wire pix_push = t_valid && !pix_full && (t_sof ? !info_full : keeping);
  wire [1:0] t_code = t_pix == 8'd0 ? 2'd0 : (t_pix == 8'd255 ? 2'd1 : 2'd2);

  wire c_valid = pix_level != '0;
  logic c_ready;
  wire [2:0] c_word = pix_mem[pix_r[PixA-1:0]];
  wire c_sof = c_word[2];
  wire c_take = c_valid && c_ready;
  wire info_t c_info = info_mem[info_r[1:0]];

  always_ff @(posedge clk) begin
    if (rst) begin
      pix_w <= '0;
      pix_r <= '0;
      info_w <= '0;
      info_r <= '0;
      info_o <= '0;
      keeping <= 1'b0;
      o_frames_cut <= '0;
      o_pix_level_max <= '0;
    end else begin
      if (pix_push) begin
        pix_mem[pix_w[PixA-1:0]] <= {t_sof, t_code};
        pix_w <= pix_w + 1'b1;
        if (t_sof) begin
          info_mem[info_w[1:0]] <= in_info;
          info_w <= info_w + 3'd1;
          keeping <= 1'b1;
        end
      end else if (t_valid && (t_sof || keeping)) begin
        keeping <= 1'b0;
        o_frames_cut <= o_frames_cut + 32'd1;
      end
      if (c_take) begin
        pix_r <= pix_r + 1'b1;
        if (c_sof) info_r <= info_r + 3'd1;
      end
      if (r_start) info_o <= info_o + 3'd1;
      if (32'(pix_level) > o_pix_level_max) o_pix_level_max <= 32'(pix_level);
    end
  end

  // ----------------------------------------------------------- components --
  logic r_valid, r_white, r_start, r_done, r_overflow;
  logic [31:0] r_id, r_area;
  logic [11:0] r_xmin, r_xmax, r_ymin, r_ymax;
  logic [11:0] r_ex[16], r_ey[16];
  /* verilator lint_off PINCONNECTEMPTY */
  components #(
      .MAX_W(MAX_W),
      .MAX_LABELS(MAX_LABELS)
  ) u_components (
      .clk,
      .rst,
      .i_width(c_info.width),
      .i_height(c_info.height),
      .in_valid(c_valid),
      .in_ready(c_ready),
      .in_sof(c_sof),
      .in_pix(c_word[1:0] == 2'd0 ? 8'd0 : (c_word[1:0] == 2'd1 ? 8'd255 : 8'd127)),
      .o_valid(r_valid),
      .o_id(r_id),
      .o_white(r_white),
      .o_area(r_area),
      .o_xmin(r_xmin),
      .o_xmax(r_xmax),
      .o_ymin(r_ymin),
      .o_ymax(r_ymax),
      .o_closed_after_row(),
      .o_ex(r_ex),
      .o_ey(r_ey),
      .o_frame_start(r_start),
      .o_frame_done(r_done),
      .o_overflow(r_overflow),
      .o_row_cycles_max,
      .o_labels_max
  );
  /* verilator lint_on PINCONNECTEMPTY */

  // The frame whose records components is sending: starts, records and ends
  // come out in order.
  logic [31:0] c_frame;
  logic c_active;
  wire c_abort = r_start && c_active;

  // --------------------------------------------- filter and record FIFO --
  typedef struct packed {
    logic mark;  // end of frame: area = candidates, id = dropped, small
    logic aborted;
    logic [31:0] frame;
    logic [31:0] id;
    logic [31:0] area;
    logic [31:0] small_n;
    logic [11:0] xmin, xmax, ymin, ymax;
    logic [15:0][11:0] ex, ey;
  } rec_t;
  rec_t rec_mem[REC_DEPTH];
  logic [RecA:0] rec_w, rec_r;
  wire [RecA:0] rec_level = rec_w - rec_r;
  logic [31:0] n_cand, n_small, n_drop;
  logic mark_lost;

  wire [23:0] box_w = 24'(r_xmax - r_xmin), box_h = 24'(r_ymax - r_ymin);
  wire [24:0] box_diag2 = 25'(box_w * box_w) + 25'(box_h * box_h);
  wire is_cand = r_valid && !r_white && r_area >= 32'(MIN_AREA);
  wire is_small = box_diag2 < 25'(MIN_SIDE * MIN_SIDE);
  wire rec_room = rec_level < (RecA + 1)'(REC_DEPTH - MarkSlots);
  wire push_rec = is_cand && !is_small && rec_room;
  wire push_mark = r_done || c_abort;

  rec_t new_rec;
  always_comb begin
    new_rec = '0;
    new_rec.mark = push_mark;
    new_rec.aborted = c_abort;
    new_rec.frame = c_frame;
    new_rec.id = push_mark ? n_drop : r_id;
    new_rec.area = push_mark ? n_cand : r_area;
    new_rec.small_n = n_small;
    new_rec.xmin = r_xmin;
    new_rec.xmax = r_xmax;
    new_rec.ymin = r_ymin;
    new_rec.ymax = r_ymax;
    for (int k = 0; k < 16; k++) begin
      new_rec.ex[k] = r_ex[k];
      new_rec.ey[k] = r_ey[k];
    end
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      c_active <= 1'b0;
      c_frame <= '0;
      rec_w <= '0;
      n_cand <= '0;
      n_small <= '0;
      n_drop <= '0;
      mark_lost <= 1'b0;
      o_rec_level_max <= '0;
    end else begin
      if (r_done) c_active <= 1'b0;
      if (r_start) begin
        c_active <= 1'b1;
        c_frame  <= info_mem[info_o[1:0]].frame;
      end
      if (is_cand) begin
        n_cand <= n_cand + 32'd1;
        if (is_small) n_small <= n_small + 32'd1;
        else if (!rec_room) n_drop <= n_drop + 32'd1;
      end
      if (push_mark) begin
        n_cand  <= '0;
        n_small <= '0;
        n_drop  <= '0;
        if (rec_level == (RecA + 1)'(REC_DEPTH)) mark_lost <= 1'b1;
      end
      if (push_rec || (push_mark && rec_level != (RecA + 1)'(REC_DEPTH))) begin
        rec_mem[rec_w[RecA-1:0]] <= new_rec;
        rec_w <= rec_w + 1'b1;
      end
      if (32'(rec_level) > o_rec_level_max) o_rec_level_max <= 32'(rec_level);
    end
  end

  // ------------------------------------------------------------ scorecard --
  wire rec_t head = rec_mem[rec_r[RecA-1:0]];
  wire have = rec_level != '0;
  logic s_ready, s_valid;
  logic [ 2:0] s_verdict;
  logic [31:0] s_frame;  // the record in the scorecard
  logic [11:0] h_ex[16], h_ey[16];
  always_comb
    for (int k = 0; k < 16; k++) begin
      h_ex[k] = head.ex[k];
      h_ey[k] = head.ey[k];
    end
  wire pop_rec = have && !head.mark && s_ready;
  // A marker waits for the last record's result (s_valid comes with s_ready).
  wire pop_mark = have && head.mark && s_ready && !s_valid;

  /* verilator lint_off PINCONNECTEMPTY */
  scorecard #(
      .MIN_SIDE(MIN_SIDE)
  ) u_scorecard (
      .clk,
      .rst,
      .in_valid(have && !head.mark),
      .in_ready(s_ready),
      .in_id(head.id),
      .in_area(head.area),
      .in_xmin(head.xmin),
      .in_xmax(head.xmax),
      .in_ymin(head.ymin),
      .in_ymax(head.ymax),
      .in_ex(h_ex),
      .in_ey(h_ey),
      .o_valid(s_valid),
      .o_id,
      .o_verdict(s_verdict),
      .o_cx,
      .o_cy,
      .o_area2,
      .o_cycles()
  );
  /* verilator lint_on PINCONNECTEMPTY */

  assign o_valid = s_valid && s_verdict == 3'd0;
  assign o_frame = s_frame;

  logic [31:0] n_quads, n_off;
  always_ff @(posedge clk) begin
    o_frame_done <= 1'b0;
    if (rst) begin
      rec_r   <= '0;
      n_quads <= '0;
      n_off   <= '0;
      s_frame <= '0;
    end else begin
      if (pop_rec || pop_mark) rec_r <= rec_r + 1'b1;
      if (pop_rec) s_frame <= head.frame;
      if (s_valid && s_verdict == 3'd0) n_quads <= n_quads + 32'd1;
      if (s_valid && s_verdict == 3'd3) n_off <= n_off + 32'd1;
      if (pop_mark) begin
        o_frame_done <= 1'b1;
        o_fd_frame <= head.frame;
        o_fd_aborted <= head.aborted;
        o_fd_candidates <= head.area;
        o_fd_small <= head.small_n;
        o_fd_dropped <= head.id;
        o_fd_quads <= n_quads;
        o_fd_off_quad <= n_off;
        n_quads <= '0;
        n_off <= '0;
      end
    end
  end

  always_ff @(posedge clk)
    if (rst) o_error <= 1'b0;
    else if (t_overrun || r_overflow || mark_lost) o_error <= 1'b1;

endmodule
