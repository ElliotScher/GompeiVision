// Testbench top: rtl/decode/decode.sv with its pixel memory.
module decode_tb (
    input  logic               clk,
    input  logic               rst,
    input  logic               wr_en,
    input  logic        [11:0] wr_x,
    input  logic        [11:0] wr_y,
    input  logic        [ 7:0] wr_data,
    input  logic        [11:0] img_w,
    input  logic        [11:0] img_h,
    input  logic        [ 1:0] max_bits,
    input  logic               in_valid,
    output logic               in_ready,
    input  logic        [31:0] in_tag,
    input  logic signed [31:0] in_h      [8],
    output logic               o_valid,
    output logic        [31:0] o_tag,
    output logic               o_read_ok,
    output logic        [35:0] o_code,
    output logic signed [31:0] o_margin,
    output logic               o_found,
    output logic        [ 9:0] o_id,
    output logic        [ 1:0] o_rotation,
    output logic        [ 1:0] o_hamming,
    output logic               o_error
);

  logic rd_en;
  logic [11:0] rd_x, rd_y;
  logic [7:0] rd_p[4];
  pixel_window u_window (.*);
  decode u_decode (.*);

endmodule
