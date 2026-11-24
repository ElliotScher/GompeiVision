// The PL half of the tag detector: a pixel stream in, and everything the RPU
// (rpu/) needs to finish the tags. Quad finder and tag finish side by side,
// both fed from the same pixels.
//
// The RPU gets rough quads and end-of-frame reports (q_*), sends read and edge
// jobs, gets their results (rd_*, ed_*), and watches p_frame / p_rows: how many
// whole rows of which frame are in the pixel store, which tells it when a job's
// rows are there.
//
// Frames of one size: i_width and i_height must hold still while jobs are in
// flight, and fit the store (i_height <= DEPTH).
module detector #(
    parameter string TABLE_DIR = "",
    parameter int MAX_W = 1280,
    parameter int DEPTH = 1024
) (
    input logic clk,
    input logic rst,
    input logic [7:0] min_white_black_diff,
    input logic [1:0] max_bits,
    // Pixels, in raster order.
    input logic [31:0] i_frame,
    input logic [11:0] i_width,
    input logic [11:0] i_height,
    input logic i_valid,
    input logic i_sof,
    input logic [7:0] i_pix,
    output logic [31:0] p_frame,
    output logic [11:0] p_rows,
    // Rough quads and frame ends.
    output logic q_valid,
    output logic [31:0] q_frame,
    output logic [11:0] q_cx[4],
    output logic [11:0] q_cy[4],
    output logic q_frame_done,
    output logic [31:0] q_fd_frame,
    output logic q_fd_aborted,
    output logic [31:0] q_fd_dropped,
    // Reads.
    input logic rd_job_valid,
    output logic rd_job_ready,
    input logic [31:0] rd_job_tag,
    input logic signed [31:0] rd_job_h[8],
    output logic rd_valid,
    output logic [31:0] rd_tag,
    output logic rd_ok,
    output logic [35:0] rd_code,
    output logic signed [31:0] rd_margin,
    output logic rd_found,
    output logic [9:0] rd_id,
    output logic [1:0] rd_rotation,
    output logic [1:0] rd_hamming,
    // Edges.
    input logic ed_job_valid,
    output logic ed_job_ready,
    input logic [31:0] ed_job_tag,
    input logic signed [31:0] a_x,
    input logic signed [31:0] a_y,
    input logic signed [15:0] d_x,
    input logic signed [15:0] d_y,
    input logic signed [15:0] n_x,
    input logic signed [15:0] n_y,
    input logic signed [31:0] t0,
    input logic [15:0] spacing,
    input logic [15:0] count,
    input logic [5:0] half_steps,
    input logic [5:0] window_steps,
    input logic check_direction,
    output logic pt_valid,
    output logic signed [31:0] pt_x,
    output logic signed [31:0] pt_y,
    output logic ed_done,
    output logic [31:0] ed_tag,
    output logic [31:0] o_frames_cut,
    output logic o_error  // sticky
);

  // Pixel coordinates, and the rows that are in.
  logic [11:0] x, y, w;
  logic active;
  wire take = i_valid && (i_sof || active);
  wire [11:0] cx = i_sof ? 12'd0 : x;
  wire [11:0] cy = i_sof ? 12'd0 : y;
  wire [11:0] cw = i_sof ? i_width : w;
  logic [11:0] h;
  always_ff @(posedge clk) begin
    if (rst) begin
      active  <= 1'b0;
      p_frame <= '0;
      p_rows  <= '0;
    end else if (take) begin
      if (i_sof) begin
        active <= 1'b1;
        w <= i_width;
        h <= i_height;
        p_frame <= i_frame;
        p_rows <= '0;
      end
      x <= cx + 12'd1;
      y <= cy;
      if (cx == cw - 12'd1) begin
        x <= '0;
        y <= cy + 12'd1;
        p_rows <= cy + 12'd1;
        if (cy == (i_sof ? i_height : h) - 12'd1) active <= 1'b0;
      end
    end
  end

  logic q_error, f_error;
  /* verilator lint_off PINCONNECTEMPTY */
  quad_finder #(
      .MAX_W(MAX_W)
  ) u_quads (
      .clk,
      .rst,
      .min_white_black_diff,
      .i_frame,
      .i_width,
      .i_height,
      .i_valid,
      .i_sof,
      .i_pix,
      .o_valid(q_valid),
      .o_frame(q_frame),
      .o_id(),
      .o_cx(q_cx),
      .o_cy(q_cy),
      .o_area2(),
      .o_frame_done(q_frame_done),
      .o_fd_frame(q_fd_frame),
      .o_fd_aborted(q_fd_aborted),
      .o_fd_candidates(),
      .o_fd_small(),
      .o_fd_dropped(q_fd_dropped),
      .o_fd_quads(),
      .o_fd_off_quad(),
      .o_frames_cut,
      .o_pix_level_max(),
      .o_rec_level_max(),
      .o_row_cycles_max(),
      .o_labels_max(),
      .o_error(q_error)
  );
  /* verilator lint_on PINCONNECTEMPTY */

  tag_finish #(
      .TABLE_DIR(TABLE_DIR),
      .MAX_W(MAX_W),
      .DEPTH(DEPTH)
  ) u_finish (
      .clk,
      .rst,
      .wr_en(take),
      .wr_x(cx),
      .wr_y(cy),
      .wr_data(i_pix),
      .img_w(i_width),
      .img_h(i_height),
      .max_bits,
      .rd_job_valid,
      .rd_job_ready,
      .rd_job_tag,
      .rd_job_h,
      .rd_valid,
      .rd_tag,
      .rd_ok,
      .rd_code,
      .rd_margin,
      .rd_found,
      .rd_id,
      .rd_rotation,
      .rd_hamming,
      .ed_job_valid,
      .ed_job_ready,
      .ed_job_tag,
      .a_x,
      .a_y,
      .d_x,
      .d_y,
      .n_x,
      .n_y,
      .t0,
      .spacing,
      .count,
      .half_steps,
      .window_steps,
      .check_direction,
      .pt_valid,
      .pt_x,
      .pt_y,
      .ed_done,
      .ed_tag,
      .o_error(f_error)
  );

  always_ff @(posedge clk)
    if (rst) o_error <= 1'b0;
    else if (q_error || f_error) o_error <= 1'b1;

endmodule
