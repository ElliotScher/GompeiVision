// What a tag needs after quad finding, around one pixel store: decode (reads)
// and the edge sampler (refinement). The RPU sends both kinds of job and gets
// results in order within each kind.
//
// Pixels have one 2x2 read port. Decode's samples can't wait, so they go
// first, and the edge sampler takes the cycles decode leaves.
module tag_finish #(
    parameter string TABLE_DIR = "",
    parameter int MAX_W = 1280,
    parameter int DEPTH = 1024  // rows of pixels kept, a power of 2
) (
    input logic clk,
    input logic rst,
    // Pixels in.
    input logic wr_en,
    input logic [11:0] wr_x,
    input logic [11:0] wr_y,
    input logic [7:0] wr_data,
    input logic [11:0] img_w,
    input logic [11:0] img_h,
    // Reads (rtl/decode/decode.sv).
    input logic [1:0] max_bits,
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
    // Edges (rtl/refine/edge_sampler.sv).
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
    output logic o_error
);

  logic d_en, e_en;
  logic [11:0] d_rx, d_ry, e_rx, e_ry;
  logic [7:0] rd_p[4];

  pixel_window #(
      .MAX_W(MAX_W),
      .DEPTH(DEPTH)
  ) u_window (
      .clk,
      .wr_en,
      .wr_x,
      .wr_y,
      .wr_data,
      .rd_en(d_en || e_en),
      .rd_x (d_en ? d_rx : e_rx),
      .rd_y (d_en ? d_ry : e_ry),
      .rd_p
  );

  decode #(
      .TABLE_DIR(TABLE_DIR)
  ) u_decode (
      .clk,
      .rst,
      .img_w,
      .img_h,
      .max_bits,
      .in_valid(rd_job_valid),
      .in_ready(rd_job_ready),
      .in_tag(rd_job_tag),
      .in_h(rd_job_h),
      .rd_en(d_en),
      .rd_x(d_rx),
      .rd_y(d_ry),
      .rd_p,
      .o_valid(rd_valid),
      .o_tag(rd_tag),
      .o_read_ok(rd_ok),
      .o_code(rd_code),
      .o_margin(rd_margin),
      .o_found(rd_found),
      .o_id(rd_id),
      .o_rotation(rd_rotation),
      .o_hamming(rd_hamming),
      .o_error
  );

  edge_sampler u_edges (
      .clk,
      .rst,
      .img_w,
      .img_h,
      .in_valid(ed_job_valid),
      .in_ready(ed_job_ready),
      .in_tag(ed_job_tag),
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
      .rd_en(e_en),
      .rd_ready(!d_en),
      .rd_x(e_rx),
      .rd_y(e_ry),
      .rd_p,
      .pt_valid,
      .pt_x,
      .pt_y,
      .done(ed_done),
      .o_tag(ed_tag)
  );

endmodule
