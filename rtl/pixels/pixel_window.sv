// Pixel store with a 2x2 read: any pixel and its right, lower and lower-right
// neighbors in one cycle, registered, from four UltraRAM banks split by the
// low bits of x and y. Each bank packs 8 of its pixels into a word (with byte
// writes), because UltraRAM is 72 bits wide: 4 x 8 blocks hold a 1280x800 frame
// and 4 x 10 the default 1024 rows. Rows live in a ring of DEPTH rows (row y in
// slot y % DEPTH), so it holds a whole small frame or the last DEPTH rows.
//
// rd_p is p00 (x, y), p10 (x+1, y), p01 (x, y+1), p11 (x+1, y+1), one cycle
// after rd_en. A neighbor past the right edge reads as anything, and so does a
// pixel written in the same cycle.
module pixel_window #(
    parameter int MAX_W = 1280,  // a multiple of 16
    parameter int DEPTH = 1024   // rows, a power of 2
) (
    input  logic        clk,
    input  logic        wr_en,
    input  logic [11:0] wr_x,
    input  logic [11:0] wr_y,
    input  logic [ 7:0] wr_data,
    input  logic        rd_en,
    input  logic [11:0] rd_x,
    input  logic [11:0] rd_y,
    output logic [ 7:0] rd_p   [4]
);

  localparam int RowW = $clog2(DEPTH) - 1;
  localparam int Words = MAX_W / 16 * DEPTH / 2;  // per bank
  localparam int AW = $clog2(Words);

  function automatic logic [AW-1:0] addr(input logic [11:0] x, input logic [11:0] y);
    return AW'(y[RowW:1]) * AW'(MAX_W / 16) + AW'(x[11:4]);
  endfunction

  // Each bank holds one of x, x+1 and one of y, y+1.
  wire [11:0] x_even = rd_x[0] ? rd_x + 12'd1 : rd_x, x_odd = rd_x[0] ? rd_x : rd_x + 12'd1;
  wire [11:0] y_even = rd_y[0] ? rd_y + 12'd1 : rd_y, y_odd = rd_y[0] ? rd_y : rd_y + 12'd1;

  logic [7:0] q[4];
  for (genvar g = 0; g < 4; g++) begin : g_bank  // g = {y[0], x[0]}
    (* ram_style = "ultra" *)
    logic [7:0][7:0] mem[Words];
    logic [7:0][7:0] word;
    logic [2:0] lane;
    wire [11:0] x = g % 2 == 0 ? x_even : x_odd;
    wire [11:0] y = g / 2 == 0 ? y_even : y_odd;
    wire here = wr_en && {wr_y[0], wr_x[0]} == 2'(g);
    always_ff @(posedge clk) begin
      for (int l = 0; l < 8; l++)
      if (here && wr_x[3:1] == 3'(l)) mem[addr(wr_x, wr_y)][l] <= wr_data;
    end
    always_ff @(posedge clk) begin
      if (rd_en) begin
        word <= mem[addr(x, y)];
        lane <= x[3:1];
      end
    end
    assign q[g] = word[lane];
  end

  logic [1:0] sel;
  always_ff @(posedge clk) if (rd_en) sel <= {rd_y[0], rd_x[0]};
  always_comb for (int k = 0; k < 4; k++) rd_p[k] = q[2'(k)^sel];

endmodule
