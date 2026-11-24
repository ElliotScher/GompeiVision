// PL projector (PLAN step 6, ahead of the tag sampler).
// Bit-exact with the golden model's FixedProjector (model/src/decode.cpp) at
// the Stage 4 widths: the RPU fits the homography from the refined corners
// and sends it in fixed point (h0..h5, pixel units, SCALE_FRAC fractional
// bits; h6, h7 with PERSP_FRAC; h8 = 1), and this block projects the 100
// sample points the tag sampler reads, in its order: AprilTag's 8 border
// patterns x 8, then the 36 data cells (the family's bit order).
//
// Sample points sit on a 1/16 grid of the unit square, (U, V) = 16 (u, v),
// integers -1..17, so each coordinate is one exact division:
//   x = round(num_x * 2^SHIFT / den),  SHIFT = PERSP_FRAC + OUT_FRAC - SCALE_FRAC
//   num_x = h0 U + h1 V + 16 h2,  den = h6 U + h7 V + 16 * 2^PERSP_FRAC
// rounded half away from zero to OUT_FRAC bits (y likewise with h3..h5). The
// RPU sends only valid projectors (den > 0 over the samples, every point
// within +-32768 px), so den > 0 here and each quotient fits QW bits.
//
// Points go straight to tag_sampler's point port (pt_valid, pt_idx, pt_x, pt_y).
// The cell layout is cells.mem from `gv-model lookup-tables`
// (+LOOKUP_TABLES=DIR or TABLE_DIR).
//
// Pipelined, one point per clock: `start` (when not busy) issues the 100
// points on consecutive cycles, and each comes out a fixed 28 cycles later in
// order, so busy ends before the last points are out. One quotient bit per
// pipeline stage.
module projector #(
    parameter string TABLE_DIR = "",
    parameter int SCALE_FRAC = 14,
    parameter int PERSP_FRAC = 24,
    parameter int OUT_FRAC = 8
) (
    input logic clk,
    input logic rst,
    input logic start,
    input logic signed [31:0] h0,
    input logic signed [31:0] h1,
    input logic signed [31:0] h2,
    input logic signed [31:0] h3,
    input logic signed [31:0] h4,
    input logic signed [31:0] h5,
    input logic signed [31:0] h6,
    input logic signed [31:0] h7,
    output logic busy,  // issuing points
    output logic pt_valid,
    output logic [6:0] pt_idx,
    output logic signed [31:0] pt_x,
    output logic signed [31:0] pt_y,  // Q.OUT_FRAC
    output logic done  // one cycle, with the 100th point
);

  localparam int SHIFT = PERSP_FRAC + OUT_FRAC - SCALE_FRAC;  // >= 0 at the Stage 4 widths
  localparam int QW = 24;  // |quotient| < 2^23
  localparam int NPTS = 100;

  logic [7:0] cell_rom[36];  // bit k's cell: x * 16 + y
`ifdef SYNTHESIS
  initial begin
    $readmemh({TABLE_DIR, "/cells.mem"}, cell_rom);
  end
`else
  initial begin
    string dir;
    dir = TABLE_DIR;
    if ($value$plusargs("LOOKUP_TABLES=%s", dir)) begin
    end
    $readmemh({dir, "/cells.mem"}, cell_rom);
  end
`endif

  // Point k's grid position. Border pattern p (AprilTag's patterns[], in cell
  // units: start (sx, sy), step along y for p < 4, along x after) gives
  // (U, V) = 2 * (start + i * step); cell (cx, cy) gives (2 cx + 1, 2 cy + 1).
  function automatic logic signed [5:0] grid_u(input logic [6:0] k);
    logic [7:0] c;
    if (k >= 7'd64) begin
      c = cell_rom[6'(k-7'd64)];
      return 6'(2 * c[7:4] + 1);
    end
    unique case (k[5:3])
      3'd0: return -6'sd1;
      3'd2: return 6'sd17;
      3'd3: return 6'sd15;
      3'd4, 3'd5, 3'd6, 3'd7: return 6'(1 + 2 * k[2:0]);
      default: return 6'sd1;
    endcase
  endfunction

  function automatic logic signed [5:0] grid_v(input logic [6:0] k);
    logic [7:0] c;
    if (k >= 7'd64) begin
      c = cell_rom[6'(k-7'd64)];
      return 6'(2 * c[3:0] + 1);
    end
    unique case (k[5:3])
      3'd0, 3'd1, 3'd2, 3'd3: return 6'(1 + 2 * k[2:0]);
      3'd4: return -6'sd1;
      3'd6: return 6'sd17;
      3'd7: return 6'sd15;
      default: return 6'sd1;
    endcase
  endfunction

  // Stage 0: the point. Coefficients hold for the whole read.
  logic signed [31:0] c0, c1, c2, c3, c4, c5, c6, c7;
  logic [6:0] k;
  always_ff @(posedge clk) begin
    if (rst) begin
      busy <= 1'b0;
    end else if (!busy) begin
      if (start) begin
        c0   <= h0;
        c1   <= h1;
        c2   <= h2;
        c3   <= h3;
        c4   <= h4;
        c5   <= h5;
        c6   <= h6;
        c7   <= h7;
        k    <= '0;
        busy <= 1'b1;
      end
    end else begin
      k <= k + 7'd1;
      if (k == 7'(NPTS - 1)) busy <= 1'b0;
    end
  end

  // Stage 1: numerators and denominator.
  wire signed [5:0] U = grid_u(k);
  wire signed [5:0] V = grid_v(k);
  logic v1;
  logic [6:0] k1;
  logic signed [47:0] num_x, num_y, den;
  always_ff @(posedge clk) begin
    v1 <= busy && !rst;
    k1 <= k;
    num_x <= 48'(c0) * 48'(U) + 48'(c1) * 48'(V) + 48'(c2) * 48'sd16;
    num_y <= 48'(c3) * 48'(U) + 48'(c4) * 48'(V) + 48'(c5) * 48'sd16;
    den <= 48'(c6) * 48'(U) + 48'(c7) * 48'(V) + (48'sd16 <<< PERSP_FRAC);
  end
  wire [47:0] abs_x = num_x < 0 ? 48'(-num_x) : 48'(num_x);
  wire [47:0] abs_y = num_y < 0 ? 48'(-num_y) : 48'(num_y);

  // Stage 2 and on: round(a / d) = floor((2a + d) / (2d)), a = |num| * 2^SHIFT,
  // a quotient bit per stage from the top. The quotient is under 2^QW, so the
  // numerator above its QW low bits starts as the remainder; the low bits shift
  // in at the bottom and the quotient's shift in behind them.
  localparam int DW = 40;  // 2 * den < 2^39
  localparam int NW = DW + QW;
  logic vld[QW+1], neg_x[QW+1], neg_y[QW+1];
  logic [6:0] idx[QW+1];
  logic [DW-1:0] rem_x[QW+1], rem_y[QW+1], dv[QW+1];
  logic [QW-1:0] low_x[QW+1], low_y[QW+1];
  wire [NW-1:0] n2_x = ((NW'(abs_x) << SHIFT) << 1) + NW'(den);
  wire [NW-1:0] n2_y = ((NW'(abs_y) << SHIFT) << 1) + NW'(den);
  // verilog_lint: waive-start always-ff-non-blocking
  always_ff @(posedge clk) begin
    vld[0] <= v1 && !rst;
    idx[0] <= k1;
    {rem_x[0], low_x[0]} <= n2_x;
    {rem_y[0], low_y[0]} <= n2_y;
    dv[0] <= DW'(den) << 1;
    neg_x[0] <= num_x < 0;
    neg_y[0] <= num_y < 0;
    for (int s = 0; s < QW; s++) begin
      automatic logic [DW-1:0] t_x = {rem_x[s][DW-2:0], low_x[s][QW-1]};
      automatic logic [DW-1:0] t_y = {rem_y[s][DW-2:0], low_y[s][QW-1]};
      vld[s+1] <= vld[s] && !rst;
      idx[s+1] <= idx[s];
      dv[s+1] <= dv[s];
      neg_x[s+1] <= neg_x[s];
      neg_y[s+1] <= neg_y[s];
      rem_x[s+1] <= t_x >= dv[s] ? t_x - dv[s] : t_x;
      rem_y[s+1] <= t_y >= dv[s] ? t_y - dv[s] : t_y;
      low_x[s+1] <= {low_x[s][QW-2:0], t_x >= dv[s]};
      low_y[s+1] <= {low_y[s][QW-2:0], t_y >= dv[s]};
    end
    pt_valid <= vld[QW] && !rst;
    pt_idx <= idx[QW];
    pt_x <= neg_x[QW] ? -32'(low_x[QW]) : 32'(low_x[QW]);
    pt_y <= neg_y[QW] ? -32'(low_y[QW]) : 32'(low_y[QW]);
    done <= vld[QW] && !rst && idx[QW] == 7'(NPTS - 1);
  end
  // verilog_lint: waive-stop always-ff-non-blocking

endmodule
