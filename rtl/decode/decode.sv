// Decode (PLAN step 6): one homography in (from the RPU), one read out.
// Projector -> tag sampler -> ID lookup, with consecutive reads overlapping.
// Results come out in order, with the job's tag.
//
// Pixels are read through one 2x2 port (rtl/pixels/pixel_window).
module decode #(
    parameter string TABLE_DIR = ""
) (
    input logic clk,
    input logic rst,
    input logic [11:0] img_w,
    input logic [11:0] img_h,
    input logic [1:0] max_bits,
    input logic in_valid,
    output logic in_ready,
    input logic [31:0] in_tag,
    input logic signed [31:0] in_h[8],  // h0..h5 Q.14, h6, h7 Q.24
    output logic rd_en,
    output logic [11:0] rd_x,
    output logic [11:0] rd_y,
    input logic [7:0] rd_p[4],
    output logic o_valid,
    output logic [31:0] o_tag,
    output logic o_read_ok,  // false: no plane, or a reversed border
    output logic [35:0] o_code,
    output logic signed [31:0] o_margin,  // Q.4
    output logic o_found,  // an ID within max_bits
    output logic [9:0] o_id,
    output logic [1:0] o_rotation,
    output logic [1:0] o_hamming,
    output logic o_error
);

  // in_ready: the sampler is nearly free, and no other read is on its way to
  // it. p_wait covers a read from its start until the sampler is past its
  // points (can_start drops), since the sampler looks free until then.
  logic p_busy, p_wait, can_start;
  logic pt_valid;
  logic [6:0] pt_idx;
  logic signed [31:0] pt_x, pt_y;
  wire start = in_valid && in_ready;
  assign in_ready = can_start && !p_busy && !p_wait;

  /* verilator lint_off PINCONNECTEMPTY */
  projector #(
      .TABLE_DIR(TABLE_DIR)
  ) u_projector (
      .clk,
      .rst,
      .start,
      .h0  (in_h[0]),
      .h1  (in_h[1]),
      .h2  (in_h[2]),
      .h3  (in_h[3]),
      .h4  (in_h[4]),
      .h5  (in_h[5]),
      .h6  (in_h[6]),
      .h7  (in_h[7]),
      .busy(p_busy),
      .pt_valid,
      .pt_idx,
      .pt_x,
      .pt_y,
      .done()
  );
  /* verilator lint_on PINCONNECTEMPTY */

  logic s_done, s_ok, s_error;
  logic [35:0] s_code;
  logic signed [31:0] s_margin;
  tag_sampler #(
      .TABLE_DIR(TABLE_DIR)
  ) u_sampler (
      .clk,
      .rst,
      .img_w,
      .img_h,
      .pt_valid,
      .pt_idx,
      .pt_x,
      .pt_y,
      .rd_en,
      .rd_x,
      .rd_y,
      .rd_p,
      .can_start,
      .done(s_done),
      .ok(s_ok),
      .code(s_code),
      .margin(s_margin),
      .o_error(s_error)
  );

  logic l_ready, l_valid, l_found;
  logic [9:0] l_id;
  logic [1:0] l_rotation, l_hamming;
  id_lookup #(
      .TABLE_DIR(TABLE_DIR)
  ) u_lookup (
      .clk,
      .rst,
      .in_valid(s_done && s_ok),
      .in_ready(l_ready),
      .in_code(s_code),
      .max_bits,
      .out_valid(l_valid),
      .out_found(l_found),
      .out_id(l_id),
      .out_rotation(l_rotation),
      .out_hamming(l_hamming)
  );

  // Tags of the reads in flight (at most 3: projecting, dividing, looking up).
  logic [31:0] tags[4];
  logic [1:0] tag_w, tag_r;
  wire finish = l_valid || (s_done && !s_ok);
  always_ff @(posedge clk) begin
    o_valid <= finish && !rst;
    if (rst) begin
      p_wait  <= 1'b0;
      tag_w   <= '0;
      tag_r   <= '0;
      o_error <= 1'b0;
    end else begin
      if (start) begin
        p_wait <= 1'b1;
        tags[tag_w] <= in_tag;
        tag_w <= tag_w + 2'd1;
      end else if (!can_start) p_wait <= 1'b0;
      if (finish) begin
        o_tag <= tags[tag_r];
        tag_r <= tag_r + 2'd1;
        o_read_ok <= l_valid;
        o_code <= s_code;
        o_margin <= s_margin;
        o_found <= l_valid && l_found;
        o_id <= l_id;
        o_rotation <= l_rotation;
        o_hamming <= l_hamming;
      end
      // A lookup takes at most 43 cycles and reads are further apart than
      // that, so these can't happen; they would lose a result.
      if (s_error || (s_done && s_ok && !l_ready) || (l_valid && s_done)) o_error <= 1'b1;
    end
  end

endmodule
