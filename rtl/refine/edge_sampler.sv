// PL edge sampler (PLAN step 5): edge jobs in, their edge points out, in
// order. Bit-exact with the golden model's sample_edge()
// (model/src/refine.cpp) at the Stage 4 widths (kStage4Widths): coordinates
// with 8 fractional bits, directions with 12, pixel values with 4.
//
// For sample i (t = t0 + i * spacing) the sampler:
//   1. places the point at = a + d * t;
//   2. reads a profile across the edge, at + n * j / 2 for j = -half..half,
//      with bilinear interpolation (skipping the sample if either end of the
//      search line leaves the image);
//   3. takes the central-difference gradient, finds its strongest lobe (of the
//      outward dark-to-bright sign if check_direction) within window_steps of
//      the middle, and extends it while 20 * score > peak;
//   4. returns at + n * offset, with offset the lobe's centroid.
// Every rounding is of one wide sum. The model rounds halves away from zero,
// which for a point inside the image (the only kind that matters) is half up.
//
// The RPU writes each job (it needs a square root and a division to set one
// up) and fits lines to the points. Pixels come from rtl/pixels/pixel_window:
// a request is taken when rd_en and rd_ready, and answered the next cycle.
//
// Pipelined: one profile sample per clock, 2 * half + 1 per point, with each
// stage working on a different point. Points waiting between the stages sit in
// 4 slots. The lobe grows both ways at once, so it needs at most 2 * half
// cycles and never holds up the profiles for long. half is 1..MAX_HALF.
module edge_sampler #(
    parameter int MAX_HALF = 13  // window up to 6 px: half_steps = ceil(2 * 6) + 1; at most 15
) (
    input logic clk,
    input logic rst,
    input logic [11:0] img_w,
    input logic [11:0] img_h,
    // The job.
    input logic in_valid,
    output logic in_ready,
    input logic [31:0] in_tag,
    input logic signed [31:0] a_x,
    input logic signed [31:0] a_y,  // Q.8
    input logic signed [15:0] d_x,
    input logic signed [15:0] d_y,  // Q.12
    input logic signed [15:0] n_x,
    input logic signed [15:0] n_y,  // Q.12
    input logic signed [31:0] t0,  // Q.8
    input logic [15:0] spacing,  // Q.8
    input logic [15:0] count,
    input logic [5:0] half_steps,
    input logic [5:0] window_steps,
    input logic check_direction,
    // Pixels.
    output logic rd_en,
    input logic rd_ready,
    output logic [11:0] rd_x,
    output logic [11:0] rd_y,
    input logic [7:0] rd_p[4],
    // Edge points, then done; both with the job's tag.
    output logic pt_valid,
    output logic signed [31:0] pt_x,
    output logic signed [31:0] pt_y,  // Q.8
    output logic done,  // one cycle, after the job's last point
    output logic [31:0] o_tag
);

  localparam int NP = 2 * MAX_HALF + 1;
  localparam int IW = $clog2(NP);  // profile index bits

  // What a point carries along; is_end marks the end of a job instead.
  typedef struct packed {
    logic               is_end;
    logic [31:0]        tag;
    logic signed [31:0] at_x,   at_y;
    logic signed [15:0] n_x,    n_y;
    logic [5:0]         half,   win;
    logic               check;
  } point_t;

  // ------------------------------------------------------------ placement --
  logic   job_v;
  point_t job;  // at_x, at_y hold the job's start corner
  logic signed [15:0] jd_x, jd_y;
  logic signed [47:0] jt;
  logic [15:0] jleft, jspacing;

  logic g1_v, g2_v, g2_take;
  point_t g1, g2;
  wire g2_ready = !g2_v || g2_take;
  wire g1_ready = !g1_v || g2_ready;
  wire job_end = job_v && jleft == '0;
  assign in_ready = !job_v || (job_end && g1_ready);

  // at = a + d * t (a and t Q.8, d Q.12).
  function automatic logic signed [31:0] place(
      input logic signed [31:0] a, input logic signed [15:0] d, input logic signed [47:0] t);
    return 32'((64'(a) * 64'sd4096 + 64'(d) * 64'(t) + 64'sd2048) >>> 12);
  endfunction
  // Profile position k half-steps out, times 32: at * 32 + n * k.
  function automatic logic signed [39:0] along32(
      input logic signed [31:0] at, input logic signed [15:0] n, input logic signed [6:0] k);
    return 40'(at) * 40'sd32 + 40'(n) * 40'(k);
  endfunction
  // Whether an end of the search line is on the image, pixel centers included.
  function automatic logic on_image(input logic signed [39:0] p32, input logic [11:0] size);
    logic signed [39:0] p;
    p = (p32 + 40'sd16) >>> 5;
    return p >= 40'sd128 && p <= (40'(size) <<< 8) - 40'sd128;
  endfunction
  wire signed [6:0] g1_h = 7'(g1.half);
  wire line_inside = on_image(
      along32(g1.at_x, g1.n_x, -g1_h), img_w
  ) && on_image(
      along32(g1.at_x, g1.n_x, g1_h), img_w
  ) && on_image(
      along32(g1.at_y, g1.n_y, -g1_h), img_h
  ) && on_image(
      along32(g1.at_y, g1.n_y, g1_h), img_h
  );

  always_ff @(posedge clk) begin
    if (rst) begin
      job_v <= 1'b0;
      g1_v  <= 1'b0;
      g2_v  <= 1'b0;
    end else begin
      if (job_v && g1_ready) begin
        jt <= jt + 48'(jspacing);
        jleft <= jleft - 16'd1;
        if (job_end) job_v <= 1'b0;
      end
      if (in_valid && in_ready) begin
        job_v <= 1'b1;
        job <= '{
            is_end: 1'b0,
            tag: in_tag,
            at_x: a_x,
            at_y: a_y,
            n_x: n_x,
            n_y: n_y,
            half: half_steps,
            win: window_steps,
            check: check_direction
        };
        jd_x <= d_x;
        jd_y <= d_y;
        jt <= 48'(t0);
        jleft <= count;
        jspacing <= spacing;
      end
      if (g1_ready) begin
        g1_v <= job_v;
        g1 <= job;
        g1.is_end <= job_end;
        g1.at_x <= place(job.at_x, jd_x, jt);
        g1.at_y <= place(job.at_y, jd_y, jt);
      end
      if (g2_ready) begin
        g2_v <= g1_v && (g1.is_end || line_inside);
        g2   <= g1;
      end
    end
  end

  // -------------------------------------------------------- profile issue --
  // Slots: filled in order at `wp`, profile done at `pa`, lobe done at `pb`.
  point_t slot[4];
  logic [2:0] wp, pa, pb;
  wire  slot_free = (wp - pb) != 3'd4;
  logic issuing;
  logic signed [39:0] pos_x, pos_y;  // times 32
  logic signed [15:0] is_nx, is_ny;
  logic [5:0] is_left;  // samples after this one
  wire signed [31:0] fx = 32'((pos_x + 40'sd16) >>> 5) - 32'sd128;
  wire signed [31:0] fy = 32'((pos_y + 40'sd16) >>> 5) - 32'sd128;
  assign rd_en = issuing;
  assign rd_x  = 12'(fx >>> 8);
  assign rd_y  = 12'(fy >>> 8);
  wire sent = rd_en && rd_ready;
  wire last_sent = sent && is_left == '0;
  assign g2_take = g2_v && slot_free && (!issuing || last_sent);
  wire signed [6:0] g2_h = 7'(g2.half);

  always_ff @(posedge clk) begin
    if (rst) begin
      issuing <= 1'b0;
      wp <= '0;
    end else begin
      if (sent) begin
        pos_x   <= pos_x + 40'(is_nx);
        pos_y   <= pos_y + 40'(is_ny);
        is_left <= is_left - 6'd1;
        if (is_left == '0) issuing <= 1'b0;
      end
      if (g2_take) begin
        slot[wp[1:0]] <= g2;
        wp <= wp + 3'd1;
        issuing <= !g2.is_end;
        pos_x <= along32(g2.at_x, g2.n_x, -g2_h);
        pos_y <= along32(g2.at_y, g2.n_y, -g2_h);
        is_nx <= g2.n_x;
        is_ny <= g2.n_y;
        is_left <= 6'(2 * g2.half);
      end
    end
  end

  // --------------------------------------------------------- sample pipe --
  // A: pixels arrive. B: the two row blends. Then the value (Q.4).
  logic a_v, a_last, b_v, b_last;
  logic [8:0] a_fx, a_fy, b_fy;  // 0..255
  logic [16:0] b_top, b_bot;
  wire [25:0] blend = 26'(b_top) * 26'(9'd256 - b_fy) + 26'(b_bot) * 26'(b_fy);
  wire [11:0] value = 12'((blend + 26'd2048) >> 12);
  always_ff @(posedge clk) begin
    a_v <= sent && !rst;
    a_last <= is_left == '0;
    a_fx <= {1'b0, fx[7:0]};
    a_fy <= {1'b0, fy[7:0]};
    b_v <= a_v && !rst;
    b_last <= a_last;
    b_fy <= a_fy;
    b_top <= 17'(rd_p[0]) * 17'(9'd256 - a_fx) + 17'(rd_p[1]) * 17'(a_fx);
    b_bot <= 17'(rd_p[2]) * 17'(9'd256 - a_fx) + 17'(rd_p[3]) * 17'(a_fx);
  end

  // ------------------------------------------------- gradients and peak --
  // Sample j gives the gradient (times 2) at j - 1.
  logic signed [12:0] score[4][NP];
  logic signed [12:0] peak[4];
  logic [5:0] best[4];
  logic [5:0] j;
  logic [11:0] v1, v2;
  wire point_t ps = slot[pa[1:0]];
  wire signed [12:0] grad = 13'({1'b0, value}) - 13'({1'b0, v2});
  wire signed [12:0] sc = (ps.check || grad >= 0) ? grad : -grad;
  wire [5:0] gj = j - 6'd1;
  wire [5:0] off_mid = gj > ps.half ? gj - ps.half : ps.half - gj;
  logic signed [12:0] pk;
  logic [5:0] bj;
  wire better = j >= 6'd2 && off_mid <= ps.win && sc > pk;
  always_ff @(posedge clk) begin
    if (rst) begin
      pa <= '0;
      j  <= '0;
      pk <= '0;
    end else if (b_v) begin
      v1 <= value;
      v2 <= v1;
      if (j >= 6'd2) score[pa[1:0]][IW'(gj)] <= sc;
      if (better) begin
        pk <= sc;
        bj <= gj;
      end
      j <= j + 6'd1;
      if (b_last) begin
        peak[pa[1:0]] <= better ? sc : pk;
        best[pa[1:0]] <= better ? gj : bj;
        pa <= pa + 3'd1;
        j <= '0;
        pk <= '0;
      end
    end else if (pa != wp && ps.is_end) begin
      pa <= pa + 3'd1;  // nothing to read for an end of job
    end
  end

  // ----------------------------------------------------------------- lobe --
  // Grows both ways at once, summing score and score * (index - half).
  logic growing;
  logic [5:0] lo, hi;
  logic signed [17:0] s0;
  logic signed [23:0] s1;
  wire point_t ls = slot[pb[1:0]];
  wire signed [12:0] lpk = peak[pb[1:0]];
  wire signed [12:0] sl = score[pb[1:0]][IW'(lo-6'd1)];
  wire signed [12:0] sr = score[pb[1:0]][IW'(hi+6'd1)];
  wire go_l = lo > 6'd1 && 18'sd20 * 18'(sl) > 18'(lpk);
  wire go_r = hi + 6'd2 < 6'(2 * ls.half + 1) && 18'sd20 * 18'(sr) > 18'(lpk);
  function automatic logic signed [23:0] weighted(input logic signed [12:0] s,
                                                  input logic [5:0] idx, input logic [5:0] half);
    return 24'(s) * (24'(idx) - 24'(half));
  endfunction
  wire have = pa != pb;
  wire lobe_drop = have && !growing && !ls.is_end && lpk <= 0;
  wire lobe_end = have && !growing && ls.is_end;
  wire lobe_point = growing && !go_l && !go_r;
  wire [5:0] lbest = best[pb[1:0]];
  always_ff @(posedge clk) begin
    if (rst) begin
      pb <= '0;
      growing <= 1'b0;
    end else begin
      if (lobe_drop || lobe_end || lobe_point) pb <= pb + 3'd1;
      if (growing) begin
        if (go_l) lo <= lo - 6'd1;
        if (go_r) hi <= hi + 6'd1;
        s0 <= s0 + (go_l ? 18'(sl) : 18'sd0) + (go_r ? 18'(sr) : 18'sd0);
        s1 <= s1 + (go_l ? weighted(
            sl, lo - 6'd1, ls.half
        ) : 24'sd0) + (go_r ? weighted(
            sr, hi + 6'd1, ls.half
        ) : 24'sd0);
        if (lobe_point) growing <= 1'b0;
      end else if (have && !ls.is_end && lpk > 0) begin
        growing <= 1'b1;
        lo <= lbest;
        hi <= lbest;
        s0 <= 18'(lpk);
        s1 <= weighted(lpk, lbest, ls.half);
      end
    end
  end

  // ------------------------------------------------------------- centroid --
  // offset (Q.8) = round(128 * s1 / s0), from floor(256 * |s1| / s0): under
  // 2^12, because the centroid is within half of the middle. A bit per stage.
  localparam int QW = 12;
  logic d_v[QW+1], d_end[QW+1], d_neg[QW+1];
  point_t d_pt[QW+1];
  logic [31:0] d_rem[QW+1], d_den[QW+1];
  logic [QW-1:0] d_q[QW+1];
  // Offset, then product, then point.
  logic o1_v, o1_end, o2_v, o2_end;
  point_t o1, o2;
  logic signed [12:0] off;
  logic signed [31:0] px, py;
  wire [12:0] half_up = (13'(d_q[QW]) + 13'd1) >> 1;
  always_ff @(posedge clk) begin
    d_v[0]   <= (lobe_point || lobe_end) && !rst;
    d_end[0] <= lobe_end;
    d_neg[0] <= s1 < 0;
    d_pt[0]  <= ls;
    d_rem[0] <= (s1 < 0 ? 32'(-s1) : 32'(s1)) << 8;
    d_den[0] <= 32'(s0);
    d_q[0]   <= '0;
    for (int s = 0; s < QW; s++) begin
      d_v[s+1]   <= d_v[s] && !rst;
      d_end[s+1] <= d_end[s];
      d_neg[s+1] <= d_neg[s];
      d_pt[s+1]  <= d_pt[s];
      d_den[s+1] <= d_den[s];
      d_rem[s+1] <= d_rem[s];
      d_q[s+1]   <= d_q[s];
      if (d_rem[s] >= (d_den[s] << (QW - 1 - s))) begin
        d_rem[s+1] <= d_rem[s] - (d_den[s] << (QW - 1 - s));
        d_q[s+1][QW-1-s] <= 1'b1;
      end
    end
    o1_v <= d_v[QW] && !rst;
    o1_end <= d_end[QW];
    o1 <= d_pt[QW];
    off <= d_neg[QW] ? -$signed(half_up) : $signed(half_up);
    o2_v <= o1_v && !rst;
    o2_end <= o1_end;
    o2 <= o1;
    px <= 32'(o1.n_x) * 32'(off);
    py <= 32'(o1.n_y) * 32'(off);
    pt_valid <= o2_v && !o2_end && !rst;
    done <= o2_v && o2_end && !rst;
    o_tag <= o2.tag;
    pt_x <= 32'((40'(o2.at_x) * 40'sd4096 + 40'(px) + 40'sd2048) >>> 12);
    pt_y <= 32'((40'(o2.at_y) * 40'sd4096 + 40'(py) + 40'sd2048) >>> 12);
  end

endmodule
