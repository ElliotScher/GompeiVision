// PL tag sampler (PLAN step 6, the part before the ID lookup).
// Bit-exact with the golden model's read_code() (model/src/decode.cpp) at the
// Stage 4 widths, given its projected sample points:
//   1. 64 border samples (AprilTag's 8 patterns x 8; the pixel containing
//      each point) fit two brightness planes, white (the ring outside the
//      border) and black (the border), v = c0*X + c1*Y + c2 in tag
//      coordinates scaled by 8 (integers -9..9). The sums and determinants
//      are exact; each coefficient is one rounded division (8 fractional bits).
//   2. A white plane darker than the black at the center means a reversed
//      border: not this family.
//   3. Each of the 36 cells: bilinear value (4 fractional bits) minus the
//      midpoint of the two planes there.
//   4. Sharpen with AprilTag's Laplacian at 0.25 (exact: 4 * s = 8 * v - sum
//      of the 4 neighbors on the 10x10 grid); bit = s > 0.
//   5. Decision margin: the smaller of the mean |s| of white and black bits
//      (each count starting at 1), rounded to 4 fractional bits.
// The code then goes to id_lookup.
//
// The 100 sample points (Q.8: border 0..63, then cells 64..99) stream in from
// the projector, in order, one per cycle or slower. Pixels come from
// rtl/pixels/pixel_window (2x2 reads, one cycle later). The cell layout (bit ->
// cell) is a ROM from `gv-model lookup-tables` (cells.mem; +LOOKUP_TABLES=DIR
// or TABLE_DIR).
//
// The plane solve overlaps the cells: one determinant per cycle, then six
// bit-serial divisions at once. `can_start` says a projector may start the
// next read: its first point must arrive at least 20 cycles later, by when the
// cell memories are free (the margin division runs on past that).
// o_error (sticky): a point arrived while the last read still needed them.
module tag_sampler #(
    parameter string TABLE_DIR = ""
) (
    input logic clk,
    input logic rst,
    input logic [11:0] img_w,
    input logic [11:0] img_h,
    input logic pt_valid,
    input logic [6:0] pt_idx,
    input logic signed [31:0] pt_x,
    input logic signed [31:0] pt_y,  // Q.8
    output logic rd_en,
    output logic [11:0] rd_x,
    output logic [11:0] rd_y,
    input logic [7:0] rd_p[4],
    output logic can_start,
    output logic done,  // one cycle; ok, code and margin are valid with it
    output logic ok,
    output logic [35:0] code,
    output logic signed [31:0] margin,  // Q.4
    output logic o_error
);

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

  // Border sample n's pattern: tag coordinates x 8, and white or black.
  // Patterns {start x, start y, step x, step y} in cells, as AprilTag's.
  function automatic void border_point(input logic [5:0] n, output logic signed [7:0] gx,
                                       output logic signed [7:0] gy, output logic white);
    logic signed [7:0] sx2, sy2, dx, dy, i;
    i = 8'(n[2:0]);
    unique case (n[5:3])
      3'd0: begin
        sx2 = -1;
        sy2 = 1;
        dx = 0;
        dy = 1;
        white = 1;
      end
      3'd1: begin
        sx2 = 1;
        sy2 = 1;
        dx = 0;
        dy = 1;
        white = 0;
      end
      3'd2: begin
        sx2 = 17;
        sy2 = 1;
        dx = 0;
        dy = 1;
        white = 1;
      end
      3'd3: begin
        sx2 = 15;
        sy2 = 1;
        dx = 0;
        dy = 1;
        white = 0;
      end
      3'd4: begin
        sx2 = 1;
        sy2 = -1;
        dx = 1;
        dy = 0;
        white = 1;
      end
      3'd5: begin
        sx2 = 1;
        sy2 = 1;
        dx = 1;
        dy = 0;
        white = 0;
      end
      3'd6: begin
        sx2 = 1;
        sy2 = 17;
        dx = 1;
        dy = 0;
        white = 1;
      end
      default: begin
        sx2 = 1;
        sy2 = 15;
        dx = 1;
        dy = 0;
        white = 0;
      end
    endcase
    gx = sx2 + 2 * i * dx - 8;  // 8 * tx, with tx = 2 * (s + i * d) / 8 - 1
    gy = sy2 + 2 * i * dy - 8;
  endfunction

  // ------------------------------------------------------ the sample pipe --
  // Request: a border sample is the pixel containing its point; a cell is
  // bilinear between pixel centers (golden bilinear()), and the neighbor past
  // an edge only has weight 0.
  wire is_cell = pt_idx >= 7'd64;
  wire signed [31:0] sx = is_cell ? pt_x - 32'sd128 : pt_x;
  wire signed [31:0] sy = is_cell ? pt_y - 32'sd128 : pt_y;
  wire signed [31:0] x1 = sx >>> 8, y1 = sy >>> 8;
  wire signed [31:0] x2 = is_cell ? (sx + 32'sd255) >>> 8 : x1;
  wire signed [31:0] y2 = is_cell ? (sy + 32'sd255) >>> 8 : y1;
  wire in_img = x1 >= 0 && y1 >= 0 && x2 < 32'(img_w) && y2 < 32'(img_h);
  assign rd_en = pt_valid;
  assign rd_x  = x1[11:0];
  assign rd_y  = y1[11:0];

  // A: pixels arrive. B: the two row blends (and the border sums). C: the value.
  logic a_v, a_in, b_v, b_in;
  logic [6:0] a_i, b_i;
  logic [8:0] a_fx, a_fy, b_fy;  // 0..255
  logic [16:0] b_top, b_bot;
  logic [7:0] b_pix;
  wire [25:0] b_blend = 26'(b_top) * 26'(9'd256 - b_fy) + 26'(b_bot) * 26'(b_fy);
  wire [11:0] b_value = 12'((b_blend + 26'd2048) >> 12);  // Q.4

  logic [11:0] val[36];
  logic val_in[36];
  logic [5:0] got;  // cells stored
  logic border_done;

  // Border sums per plane: xx xy x yy y 1 xv yv v.
  logic signed [18:0] sums[2][9];
  logic signed [7:0] gx, gy;
  logic white;
  always_comb border_point(b_i[5:0], gx, gy, white);
  wire signed [18:0] bx19 = 19'(gx), by19 = 19'(gy), bv19 = 19'(b_pix);
  wire bp = !white;

  // The read's progress.
  typedef enum logic [2:0] {
    S_IDLE,
    S_DET,
    S_DIV,
    S_WAIT,
    S_SUB,
    S_SHARPEN
  } state_t;
  state_t st;
  wire first = pt_valid && pt_idx == 7'd0;

  always_ff @(posedge clk) begin
    border_done <= 1'b0;
    a_v <= pt_valid && !rst;
    a_i <= pt_idx;
    a_in <= in_img;
    a_fx <= {1'b0, sx[7:0]};
    a_fy <= {1'b0, sy[7:0]};
    b_v <= a_v && !rst;
    b_i <= a_i;
    b_in <= a_in;
    b_fy <= a_fy;
    b_pix <= rd_p[0];
    b_top <= 17'(rd_p[0]) * 17'(9'd256 - a_fx) + 17'(rd_p[1]) * 17'(a_fx);
    b_bot <= 17'(rd_p[2]) * 17'(9'd256 - a_fx) + 17'(rd_p[3]) * 17'(a_fx);
    if (rst) begin
      got <= '0;
    end else begin
      if (first) begin
        got <= '0;
        for (int p = 0; p < 2; p++) for (int i = 0; i < 9; i++) sums[p][i] <= '0;
      end
      if (b_v && b_i < 7'd64) begin
        if (b_in) begin
          sums[bp][0] <= sums[bp][0] + bx19 * bx19;
          sums[bp][1] <= sums[bp][1] + bx19 * by19;
          sums[bp][2] <= sums[bp][2] + bx19;
          sums[bp][3] <= sums[bp][3] + by19 * by19;
          sums[bp][4] <= sums[bp][4] + by19;
          sums[bp][5] <= sums[bp][5] + 19'sd1;
          sums[bp][6] <= sums[bp][6] + bx19 * bv19;
          sums[bp][7] <= sums[bp][7] + by19 * bv19;
          sums[bp][8] <= sums[bp][8] + bv19;
        end
        border_done <= b_i == 7'd63;
      end
      if (b_v && b_i >= 7'd64) begin
        val[6'(b_i-7'd64)] <= b_value;
        val_in[6'(b_i-7'd64)] <= b_in;
        got <= got + 6'd1;
      end
    end
  end

  // ---------------------------------------------------- the plane solve --
  // Cramer's rule on each plane's normal equations, as Plane::solve(). Step
  // {plane, col}: column `col` of the matrix replaced by the right side, or
  // for col 3 the matrix itself (the denominator).
  logic [2:0] step, step1, step2;
  logic det_v, det_v1, det_v2;
  logic signed [18:0] m[9], b[3];
  always_comb begin
    for (int r = 0; r < 3; r++) b[r] = sums[step[2]][6+r];
    m = '{
        sums[step[2]][0],
        sums[step[2]][1],
        sums[step[2]][2],
        sums[step[2]][1],
        sums[step[2]][3],
        sums[step[2]][4],
        sums[step[2]][2],
        sums[step[2]][4],
        sums[step[2]][5]
    };
    for (int r = 0; r < 3; r++)
    for (int col = 0; col < 3; col++) if (step[1:0] == 2'(col)) m[r*3+col] = b[r];
  end
  // Products of natural-width operands: casting them wide first makes tools
  // build far bigger multipliers.
  wire signed [37:0] p48 = m[4] * m[8], p57 = m[5] * m[7], p38 = m[3] * m[8];
  wire signed [37:0] p56 = m[5] * m[6], p37 = m[3] * m[7], p46 = m[4] * m[6];
  logic signed [37:0] minor0, minor1, minor2;
  logic signed [18:0] m0, m1, m2;
  wire signed  [57:0] det0 = m0 * minor0, det1 = m1 * minor1, det2 = m2 * minor2;
  logic signed [57:0] det;
  logic signed [57:0] num[2][3], dens[2];

  // The serial dividers: round(|num| * 256 / |den|), halves up, a bit of
  // the numerator per cycle (it is under 2^53), keeping 32 quotient bits.
  localparam int DivBits = 54;
  logic [DivBits-1:0] dnum[2][3];
  logic [DivBits:0] drem[2][3];
  logic [31:0] dq[2][3];
  logic [6:0] dcnt;
  logic signed [31:0] coef[2][3];

  function automatic logic [57:0] mag(input logic signed [57:0] v);
    return v < 0 ? 58'(-v) : 58'(v);
  endfunction

  // ------------------------------------------------- cells, bits, margin --
  logic [5:0] k;
  wire  [3:0] bx = cell_rom[k][7:4], by = cell_rom[k][3:0];
  // Cell values (Q.9) by {y, x}. The data cells are x, y = 1..6, so every
  // neighbor is 0..7, and those on the rim are 0. Five copies, for five reads.
  logic signed [31:0] grid0[64], grid1[64], grid2[64], grid3[64], grid4[64];
  wire [5:0] gi = {by[2:0], bx[2:0]};
  wire signed [4:0] gx8 = 5'(2 * bx) - 5'sd7, gy8 = 5'(2 * by) - 5'sd7;  // -5..5
  wire signed [31:0] cx = coef[0][0] + coef[1][0], cy = coef[0][1] + coef[1][1];
  wire signed [31:0] cx_x = cx * gx8, cy_y = cy * gy8;
  wire signed [31:0] thr = cx_x + cy_y + coef[0][2] + coef[1][2];
  // S_SUB: value (Q.9) = bilinear (Q.4) - midpoint of the planes at the cell.
  wire grid_we = st == S_SUB;
  wire signed [31:0] grid_wd = val_in[k] ? 32'(val[k]) * 32 - thr : '0;
  always_ff @(posedge clk)
    if (grid_we) begin
      grid0[gi] <= grid_wd;
      grid1[gi] <= grid_wd;
      grid2[gi] <= grid_wd;
      grid3[gi] <= grid_wd;
      grid4[gi] <= grid_wd;
    end
  // 4 * sharpened.
  wire signed [31:0] up = by[2:0] == 3'd1 ? '0 : grid1[gi-6'd8];
  wire signed [31:0] down = by[2:0] == 3'd6 ? '0 : grid2[gi+6'd8];
  wire signed [31:0] left = bx[2:0] == 3'd1 ? '0 : grid3[gi-6'd1];
  wire signed [31:0] right = bx[2:0] == 3'd6 ? '0 : grid4[gi+6'd1];
  wire signed [31:0] s4 = 8 * grid0[gi] - up - down - left - right;

  logic [35:0] bits;
  logic signed [39:0] w_sum, b_sum;
  logic [5:0] w_cnt, b_cnt;  // each starts at 1
  wire signed [ 6:0] w_cnt7 = {1'b0, w_cnt}, b_cnt7 = {1'b0, b_cnt};
  wire signed [46:0] w_cross = w_sum * b_cnt7, b_cross = b_sum * w_cnt7;
  // round(sum / (128 * cnt)) = floor(((sum + 64 * cnt) / 128) / cnt): serial.
  logic mdiv, mload;
  logic [32:0] mnum;
  logic [6:0] mrem;
  logic [5:0] mden;
  logic [5:0] mcnt;
  logic [32:0] mq;
  logic [35:0] mcode;
  wire [6:0] mrem_next = {mrem[5:0], mnum[32]};
  wire mge = mrem_next >= {1'b0, mden};
  wire w_less = w_cross <= b_cross;
  wire [39:0] msum = w_less ? w_sum : b_sum;
  wire [5:0] mcount = w_less ? w_cnt : b_cnt;

  assign can_start = st == S_IDLE || (st == S_SHARPEN && k >= 6'd18);

  // `c` and the loop temporaries are blocking locals.
  // verilog_lint: waive-start always-ff-non-blocking
  always_ff @(posedge clk) begin
    done <= 1'b0;
    mload <= 1'b0;
    det_v1 <= det_v;
    det_v2 <= det_v1;
    step1 <= step;
    step2 <= step1;
    minor0 <= p48 - p57;
    minor1 <= p38 - p56;
    minor2 <= p37 - p46;
    m0 <= m[0];
    m1 <= m[1];
    m2 <= m[2];
    det <= det0 - det1 + det2;
    if (det_v2) begin
      if (step2[1:0] == 2'd3) dens[step2[2]] <= det;
      else num[step2[2]][step2[1:0]] <= det;
    end
    if (rst) begin
      st <= S_IDLE;
      det_v <= 1'b0;
      mdiv <= 1'b0;
      o_error <= 1'b0;
    end else begin
      if (first && st != S_IDLE) o_error <= 1'b1;
      unique case (st)
        S_IDLE:
        if (border_done) begin
          step <= '0;
          det_v <= 1'b1;
          st <= S_DET;
        end
        S_DET: begin
          if (det_v) begin
            step <= step + 3'd1;
            if (step == 3'd7) det_v <= 1'b0;
          end else if (!det_v1 && !det_v2) begin
            for (int p = 0; p < 2; p++)
            for (int i = 0; i < 3; i++) begin
              dnum[p][i] <= DivBits'(mag(num[p][i])) * DivBits'(512) + DivBits'(mag(dens[p]));
              drem[p][i] <= '0;
              dq[p][i]   <= '0;
            end
            dcnt <= '0;
            st   <= S_DIV;
          end
        end
        S_DIV: begin
          for (int p = 0; p < 2; p++)
          for (int i = 0; i < 3; i++) begin
            automatic logic [DivBits:0] r = {drem[p][i][DivBits-1:0], dnum[p][i][DivBits-1]};
            automatic logic [DivBits:0] d2 = (DivBits + 1)'(mag(dens[p])) << 1;
            dnum[p][i] <= dnum[p][i] << 1;
            drem[p][i] <= r >= d2 ? r - d2 : r;
            dq[p][i]   <= {dq[p][i][30:0], r >= d2};
          end
          dcnt <= dcnt + 7'd1;
          if (dcnt == 7'(DivBits - 1)) st <= S_WAIT;
        end
        S_WAIT: begin
          automatic logic signed [31:0] c[2][3];
          automatic logic fail = 1'b0;
          for (int p = 0; p < 2; p++) begin
            if (sums[p][5] < 19'sd3 || dens[p] == 0) fail = 1'b1;
            for (int i = 0; i < 3; i++)
            c[p][i] = (num[p][i] < 0) != (dens[p] < 0) ? -$signed(dq[p][i]) : $signed(dq[p][i]);
          end
          coef <= c;
          k <= '0;
          if (got == 6'd36) begin
            if (fail || c[0][2] - c[1][2] < 0) begin  // no plane, or a reversed border
              ok <= 1'b0;
              code <= '0;
              margin <= '0;
              done <= 1'b1;
              st <= S_IDLE;
            end else begin
              st <= S_SUB;
            end
          end
        end
        S_SUB: begin
          k <= k + 6'd1;
          if (k == 6'd35) begin
            k <= '0;
            w_sum <= '0;
            b_sum <= '0;
            w_cnt <= 6'd1;
            b_cnt <= 6'd1;
            st <= S_SHARPEN;
          end
        end
        S_SHARPEN: begin
          bits <= {bits[34:0], s4 > 0};
          if (s4 > 0) begin
            w_sum <= w_sum + 40'(s4);
            w_cnt <= w_cnt + 6'd1;
          end else begin
            b_sum <= b_sum - 40'(s4);
            b_cnt <= b_cnt + 6'd1;
          end
          k <= k + 6'd1;
          if (k == 6'd35) begin
            mload <= 1'b1;
            st <= S_IDLE;
          end
        end
        default: st <= S_IDLE;
      endcase

      // The smaller mean |s| is in units of 2^-11; the margin is Q.4.
      if (mload) begin
        mnum  <= 33'((msum + 40'(mcount) * 40'd64) >> 7);
        mden  <= mcount;
        mrem  <= '0;
        mcnt  <= '0;
        mcode <= bits;
        mdiv  <= 1'b1;
      end
      if (mdiv) begin
        mnum <= mnum << 1;
        mrem <= mge ? mrem_next - {1'b0, mden} : mrem_next;
        mq   <= {mq[31:0], mge};
        mcnt <= mcnt + 6'd1;
        if (mcnt == 6'd32) begin
          mdiv <= 1'b0;
          ok <= 1'b1;
          code <= mcode;
          margin <= 32'({mq[31:0], mge});
          done <= 1'b1;
        end
      end
    end
  end
  // verilog_lint: waive-stop always-ff-non-blocking

endmodule
