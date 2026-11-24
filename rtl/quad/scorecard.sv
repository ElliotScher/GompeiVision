// Quad finding 4B, the corner scorecard (PLAN step 4B), first version.
// Bit-exact with the golden model's scorecard_quad() (model/src/quad.cpp):
// one closed black component's record in (from rtl/components), its verdict
// and rough quad out (to the RPU, which grows it and fits the quick decode's
// homography).
//
// Every quantity is an exact integer: differences of pixel centers are
// integers, so squared distances, cross products and twice the quad's area
// are exact. The only rounding is each side length, floor(256 * length) from
// a bit-serial integer square root, and every comparison that uses one is
// cross-multiplied. The steps, one item per cycle:
//   1. START:  find an extreme that doesn't merge with its predecessor (a
//              component whose 16 extremes all merge is a dot).
//   2. MERGE:  walk the 16 extremes cyclically from there; an extreme within
//              MERGE_SQ (px^2) of its cluster's first one joins the cluster,
//              which keeps the point farthest from the bounding-box center.
//   3. SEARCH: every 4 of the n merged points in cyclic order, largest twice
//              area wins (first in a < b < d < e order on a tie).
//   4. SIDES:  each side at least MIN_SIDE px, and its length in Q.8.
//   5. OFFQ:   no merged point more than max(MAX_OUTSIDE_Q8 / 256 px, mean
//              side / MAX_OUTSIDE_DIV) outside any side.
//   6. FILL:   blob area / (quad area + perimeter / 2 + 1) within
//              MIN_FILL_PCT..MAX_FILL_PCT percent.
//
// Verdicts: 0 ok, 1 too few corners, 2 too small, 3 extremes off quad (the
// 4C tracer's input), 4 fill out of range. Corners are integer pixel indices
// (the extreme pixels; their centers are +0.5) in direction order, clockwise
// on screen, valid unless the verdict is 1.
//
// Limits of this version: the search is one candidate per cycle with four
// multipliers, so up to C(16, 4) = 1,820 cycles for a component with 16
// distinct extremes (tags merge to 4-8). A table of the 120 pairwise cross
// products would make it adders only, and the max-area quad of a convex
// polygon doesn't need the exhaustive search at all; that's the throughput
// pass, sized on recorded frames.
module scorecard #(
    parameter int MERGE_SQ = 2,
    parameter int MIN_SIDE = 8,
    parameter int MAX_OUTSIDE_Q8 = 512,
    parameter int MAX_OUTSIDE_DIV = 20,
    parameter int MIN_FILL_PCT = 25,
    parameter int MAX_FILL_PCT = 95
) (
    input logic clk,
    input logic rst,

    // One component record, as rtl/components emits it.
    input  logic        in_valid,
    output logic        in_ready,
    input  logic [31:0] in_id,
    input  logic [31:0] in_area,
    input  logic [11:0] in_xmin,
    input  logic [11:0] in_xmax,
    input  logic [11:0] in_ymin,
    input  logic [11:0] in_ymax,
    input  logic [11:0] in_ex   [16],
    input  logic [11:0] in_ey   [16],

    output logic               o_valid,       // one cycle
    output logic        [31:0] o_id,
    output logic        [ 2:0] o_verdict,
    output logic        [11:0] o_cx     [4],
    output logic        [11:0] o_cy     [4],
    output logic signed [31:0] o_area2,       // twice the quad's area, px^2
    output logic        [15:0] o_cycles       // this record's processing time
);

  localparam logic [2:0] VOk = 3'd0, VFew = 3'd1, VSmall = 3'd2, VOff = 3'd3, VFill = 3'd4;
  localparam int SqW = 44;  // d2 < 2^27, shifted left 16
  localparam int RootW = SqW / 2;
  localparam longint DIV4 = 4 * longint'(MAX_OUTSIDE_DIV);

  typedef enum logic [3:0] {
    S_IDLE,
    S_START,
    S_MERGE,
    S_MERGE_END,
    S_SEARCH,
    S_CORNERS,
    S_SIDES,
    S_SQRT,
    S_OFFQ,
    S_FILL,
    S_DONE
  } state_t;
  state_t state;

  // The record.
  logic [31:0] id_r, area_r;
  logic signed [13:0] cx2, cy2;  // bounding-box center, doubled
  logic [11:0] rx[16], ry[16];

  // Merged points.
  logic [11:0] px[16], py[16];
  logic [4:0] n;
  logic [4:0] i;
  logic [3:0] start_i;
  logic [11:0] ax, ay, kx, ky;  // the cluster's anchor and kept point

  // Search.
  logic [3:0] a, b, d, e;
  logic signed [31:0] best;
  logic [3:0] pick[4];

  // The quad and its checks.
  logic [11:0] qx[4], qy[4];
  logic [1:0] k;
  logic [3:0] j;
  logic [RootW-1:0] len_q8[4];
  logic [23:0] perim_q8;
  logic [SqW-1:0] sq_v, sq_rem;
  logic [RootW-1:0] sq_root;
  logic [4:0] sq_step;
  logic [2:0] verdict;
  logic [15:0] cycles;

  // Products below widen their operands first: Verilator evaluates 28'(dx * dx)
  // at 28 bits, but a tool may take a size cast's operand at dx's own width.
  function automatic logic signed [13:0] diff(input logic [11:0] u, input logic [11:0] v);
    return 14'(signed'({2'b00, u}) - signed'({2'b00, v}));
  endfunction

  function automatic logic [27:0] dist_sq(input logic [11:0] ux, input logic [11:0] uy,
                                          input logic [11:0] vx, input logic [11:0] vy);
    logic signed [13:0] dx, dy;
    dx = diff(ux, vx);
    dy = diff(uy, vy);
    return 28'(28'(dx) * 28'(dx)) + 28'(28'(dy) * 28'(dy));
  endfunction

  function automatic logic near(input logic [11:0] ux, input logic [11:0] uy, input logic [11:0] vx,
                                input logic [11:0] vy);
    return dist_sq(ux, uy, vx, vy) <= 28'(MERGE_SQ);
  endfunction

  // Squared distance of a pixel center from the bounding-box center, doubled.
  function automatic logic [29:0] reach(input logic [11:0] x, input logic [11:0] y,
                                        input logic signed [13:0] c2x,
                                        input logic signed [13:0] c2y);
    logic signed [14:0] dx, dy;
    dx = 15'(signed'({2'b00, x, 1'b1}) - 15'(c2x));
    dy = 15'(signed'({2'b00, y, 1'b1}) - 15'(c2y));
    return 30'(30'(dx) * 30'(dx)) + 30'(30'(dy) * 30'(dy));
  endfunction

  // cross_prod(o, u, v) = (u - o) x (v - o).
  function automatic logic signed [28:0] cross_prod(input logic [11:0] ox, input logic [11:0] oy,
                                                    input logic [11:0] ux, input logic [11:0] uy,
                                                    input logic [11:0] vx, input logic [11:0] vy);
    logic signed [13:0] ux_o, uy_o, vx_o, vy_o;
    ux_o = diff(ux, ox);
    uy_o = diff(uy, oy);
    vx_o = diff(vx, ox);
    vy_o = diff(vy, oy);
    return 29'(ux_o) * 29'(vy_o) - 29'(uy_o) * 29'(vx_o);
  endfunction

  // MERGE: the current extreme.
  logic [3:0] mi;
  logic [11:0] mx, my;
  assign mi = start_i + i[3:0];
  assign mx = rx[mi];
  assign my = ry[mi];

  // SEARCH: twice the area of (a, b, d, e).
  logic signed [29:0] cand_signed;
  logic signed [31:0] cand_area2;
  assign cand_signed = 30'(cross_prod(
      px[a], py[a], px[b], py[b], px[d], py[d]
  )) + 30'(cross_prod(
      px[a], py[a], px[d], py[d], px[e], py[e]
  ));
  assign cand_area2 = cand_signed < 0 ? -32'(cand_signed) : 32'(cand_signed);

  // SIDES: side k.
  logic [ 1:0] k1;
  logic [27:0] side_d2;
  assign k1 = k + 2'd1;
  assign side_d2 = dist_sq(qx[k], qy[k], qx[k1], qy[k1]);

  // SQRT: one result bit per cycle.
  logic [SqW-1:0] sq_rem_next, sq_trial;
  assign sq_rem_next = {sq_rem[SqW-3:0], sq_v[SqW-1-:2]};
  assign sq_trial = SqW'({sq_root, 2'b01});

  // OFFQ: point j against side k: -cross * 4 * DIV * 2^16 > M * len_q8.
  logic signed [63:0] off_lhs, off_rhs, m_q8;
  assign m_q8 = 64'(longint'(MAX_OUTSIDE_Q8) * DIV4) > 64'(perim_q8) ?
      64'(longint'(MAX_OUTSIDE_Q8) * DIV4) : 64'(perim_q8);
  assign off_lhs = -64'(cross_prod(qx[k], qy[k], qx[k1], qy[k1], px[j], py[j])) * (DIV4 * 65536);
  assign off_rhs = m_q8 * 64'(len_q8[k]);

  // FILL: blob * 512 * 100 against the percentages of
  // 256 * twice_area + perimeter_q8 + 512.
  logic signed [63:0] fill_den, fill_blob;
  assign fill_den  = 64'(best) * 256 + 64'(perim_q8) + 64'd512;
  assign fill_blob = 64'(area_r) * 51200;

  assign in_ready  = state == S_IDLE;

  always_ff @(posedge clk) begin
    o_valid <= 1'b0;
    if (state != S_IDLE) cycles <= cycles + 16'd1;
    if (rst) begin
      state <= S_IDLE;
    end else begin
      unique case (state)
        S_IDLE:
        if (in_valid) begin
          id_r <= in_id;
          area_r <= in_area;
          cx2 <= 14'(in_xmin) + 14'(in_xmax) + 14'd1;
          cy2 <= 14'(in_ymin) + 14'(in_ymax) + 14'd1;
          rx <= in_ex;
          ry <= in_ey;
          i <= '0;
          n <= '0;
          cycles <= 16'd1;
          state <= S_START;
        end

        S_START:
        if (!near(rx[i[3:0]], ry[i[3:0]], rx[i[3:0]-4'd1], ry[i[3:0]-4'd1])) begin
          start_i <= i[3:0];
          i <= '0;
          state <= S_MERGE;
        end else if (i == 5'd15) begin
          verdict <= VFew;
          state   <= S_DONE;
        end else begin
          i <= i + 5'd1;
        end

        S_MERGE: begin
          if (i != 0 && near(mx, my, ax, ay)) begin
            if (reach(mx, my, cx2, cy2) > reach(kx, ky, cx2, cy2)) begin
              kx <= mx;
              ky <= my;
            end
          end else begin
            if (i != 0) begin
              px[n[3:0]] <= kx;
              py[n[3:0]] <= ky;
              n <= n + 5'd1;
            end
            ax <= mx;
            ay <= my;
            kx <= mx;
            ky <= my;
          end
          i <= i + 5'd1;
          if (i == 5'd15) state <= S_MERGE_END;
        end

        S_MERGE_END: begin
          px[n[3:0]] <= kx;
          py[n[3:0]] <= ky;
          n <= n + 5'd1;
          if (n + 5'd1 < 5'd4) begin
            verdict <= VFew;
            state   <= S_DONE;
          end else begin
            a <= 4'd0;
            b <= 4'd1;
            d <= 4'd2;
            e <= 4'd3;
            best <= -32'sd1;
            state <= S_SEARCH;
          end
        end

        S_SEARCH: begin
          if (cand_area2 > best) begin
            best <= cand_area2;
            pick <= '{a, b, d, e};
          end
          if (5'(e) < n - 5'd1) begin
            e <= e + 4'd1;
          end else if (5'(d) < n - 5'd2) begin
            d <= d + 4'd1;
            e <= d + 4'd2;
          end else if (5'(b) < n - 5'd3) begin
            b <= b + 4'd1;
            d <= b + 4'd2;
            e <= b + 4'd3;
          end else if (5'(a) < n - 5'd4) begin
            a <= a + 4'd1;
            b <= a + 4'd2;
            d <= a + 4'd3;
            e <= a + 4'd4;
          end else begin
            state <= S_CORNERS;
          end
        end

        S_CORNERS: begin
          for (int c = 0; c < 4; c++) begin
            qx[c] <= px[pick[c]];
            qy[c] <= py[pick[c]];
          end
          k <= 2'd0;
          perim_q8 <= '0;
          state <= S_SIDES;
        end

        S_SIDES:
        if (side_d2 < 28'(MIN_SIDE * MIN_SIDE)) begin
          verdict <= VSmall;
          state   <= S_DONE;
        end else begin
          sq_v <= SqW'(side_d2) << 16;
          sq_rem <= '0;
          sq_root <= '0;
          sq_step <= '0;
          state <= S_SQRT;
        end

        S_SQRT: begin
          sq_v <= sq_v << 2;
          if (sq_rem_next >= sq_trial) begin
            sq_rem  <= sq_rem_next - sq_trial;
            sq_root <= {sq_root[RootW-2:0], 1'b1};
          end else begin
            sq_rem  <= sq_rem_next;
            sq_root <= {sq_root[RootW-2:0], 1'b0};
          end
          sq_step <= sq_step + 5'd1;
          if (sq_step == 5'(RootW - 1)) begin
            len_q8[k] <= {sq_root[RootW-2:0], sq_rem_next >= sq_trial};
            perim_q8 <= perim_q8 + 24'({sq_root[RootW-2:0], sq_rem_next >= sq_trial});
            k <= k + 2'd1;
            if (k == 2'd3) begin
              j <= '0;
              state <= S_OFFQ;
            end else begin
              state <= S_SIDES;
            end
          end
        end

        S_OFFQ:
        if (off_lhs > off_rhs) begin
          verdict <= VOff;
          state   <= S_DONE;
        end else begin
          k <= k + 2'd1;
          if (k == 2'd3) begin
            j <= j + 4'd1;
            if (5'(j) == n - 5'd1) state <= S_FILL;
          end
        end

        S_FILL: begin
          verdict <= fill_blob < 64'(MIN_FILL_PCT) * fill_den ||
              fill_blob > 64'(MAX_FILL_PCT) * fill_den ? VFill : VOk;
          state <= S_DONE;
        end

        S_DONE: begin
          o_valid <= 1'b1;
          o_id <= id_r;
          o_verdict <= verdict;
          o_cx <= qx;
          o_cy <= qy;
          o_area2 <= best;
          o_cycles <= cycles;
          state <= S_IDLE;
        end

        default: state <= S_IDLE;
      endcase
    end
  end

endmodule
