// Adaptive threshold (PLAN step 2), streaming. Bit-exact with the golden
// model model/include/gv/model/threshold.h (and so AprilTag's threshold()):
//
//   1. min/max per full 4x4 tile;
//   2. each tile's min/max dilated over its 3x3 tile neighborhood (clipped);
//   3. pixels in full tiles: 127 if max - min < min_white_black_diff, else
//      255 if v > min + (max - min) / 2, else 0. Pixels in the partial right
//      and bottom tiles use the nearest full tile and are never 127.
//
// Input: one pixel per clock in raster order. i_sof marks the frame's first
// pixel and latches i_width and i_height (width 4..MAX_W, height 4..4095).
//
// A pixel's dilated stats need the tile row below it, so output row y starts
// once input rows up to the end of tile row min(y/4 + 1, last) are in: up to
// 12 rows later. Rows wait in a 16-row ring. The output side runs on its own
// at one pixel per clock (plus 3 cycles per row), including after the last
// input row, so the frame's tail drains during vertical blanking.
// Requirements, flagged on o_overrun (sticky until reset) if broken:
//   - on average at least 3 idle cycles per input row (horizontal blanking);
//   - about 12 rows of vertical blanking before the next frame.
//
// Memory: 16 x MAX_W pixel bytes, plus 4 tile rows x MAX_W/4 x (min, max).
module threshold #(
    parameter int MAX_W = 1280  // widest frame; heights up to 4095
) (
    input  logic        clk,
    input  logic        rst,
    input  logic [ 7:0] min_white_black_diff,
    input  logic [11:0] i_width,
    input  logic [11:0] i_height,
    input  logic        i_valid,
    input  logic        i_sof,
    input  logic [ 7:0] i_pix,
    output logic        o_valid,
    output logic        o_sof,
    output logic        o_eol,
    output logic [11:0] o_x,
    output logic [11:0] o_y,
    output logic [ 7:0] o_pix,
    output logic        o_overrun
);

  localparam int TWMAX = MAX_W / 4;
  localparam int TA = $clog2(TWMAX);  // tile column address bits
  localparam int PA = $clog2(16 * MAX_W);  // pixel ring address bits

  // ------------------------------------------------------- frame geometry --
  logic [11:0] width, height;  // latched at SOF
  wire [ 9:0] tiles_x = width[11:2];  // full tiles per row
  wire [ 9:0] tiles_y = height[11:2];  // full tile rows
  wire [11:0] full_w = {tiles_x, 2'b00};
  wire [11:0] full_h = {tiles_y, 2'b00};

  // ---------------------------------------------------------------- input --
  logic [11:0] in_x, in_y;
  logic [11:0] rows_done;  // input rows completely written
  logic in_active;
  logic [11:0] oy;  // output row (the overrun check compares against it)
  logic frame_out;  // a frame is being output

  logic [7:0] pix_mem[16 * MAX_W];  // row y in slot y % 16
  logic [7:0] tmin_mem[4][TWMAX];  // tile row r in slot r % 4
  logic [7:0] tmax_mem[4][TWMAX];

  function automatic logic [PA-1:0] pix_addr(input logic [11:0] x, input logic [3:0] slot);
    return PA'(slot) * PA'(MAX_W) + PA'(x);
  endfunction

  // The current pixel's coordinates: (0, 0) on SOF, else the counters.
  wire take = i_valid && (i_sof || in_active);
  wire [11:0] cx = i_sof ? 12'd0 : in_x;
  wire [11:0] cy = i_sof ? 12'd0 : in_y;
  wire [11:0] cw = i_sof ? i_width : width;
  wire [11:0] ch = i_sof ? i_height : height;
  // Pixel (0, 0) is always in a full tile (width, height >= 4); elsewhere
  // use the latched geometry.
  wire c_full = i_sof || ((cx < full_w) && (cy < full_h));

  logic [7:0] seg_min, seg_max;  // this row's part of the current tile
  logic [7:0] acc_min_q, acc_max_q;  // earlier rows of the current tile
  wire [7:0] seg_min_n = (cx[1:0] == 2'd0 || i_pix < seg_min) ? i_pix : seg_min;
  wire [7:0] seg_max_n = (cx[1:0] == 2'd0 || i_pix > seg_max) ? i_pix : seg_max;
  wire [1:0] c_slot = cy[3:2];
  wire [TA-1:0] c_tx = TA'(cx[11:2]);

  always_ff @(posedge clk) begin
    if (rst) begin
      in_active <= 1'b0;
      rows_done <= '0;
      o_overrun <= 1'b0;
      in_x <= '0;
      in_y <= '0;
      width <= 12'd4;
      height <= 12'd4;
    end else if (take) begin
      if (i_sof) begin
        if (frame_out) o_overrun <= 1'b1;  // previous frame still draining
        width <= i_width;
        height <= i_height;
        rows_done <= '0;
        in_active <= 1'b1;
      end
      pix_mem[pix_addr(cx, cy[3:0])] <= i_pix;
      if (!i_sof && cx == '0 && frame_out && cy >= oy + 12'd16) o_overrun <= 1'b1;
      if (cx == cw - 1'b1) begin
        in_x <= '0;
        in_y <= cy + 1'b1;
        rows_done <= cy + 1'b1;
        if (cy == ch - 1'b1) in_active <= 1'b0;
      end else begin
        in_x <= cx + 1'b1;
        in_y <= cy;
      end
      // Tile statistics.
      if (c_full) begin
        seg_min <= seg_min_n;
        seg_max <= seg_max_n;
        if (cx[1:0] == 2'd0) begin
          acc_min_q <= tmin_mem[c_slot][c_tx];
          acc_max_q <= tmax_mem[c_slot][c_tx];
        end
        if (cx[1:0] == 2'd3) begin
          if (cy[1:0] == 2'd0) begin
            tmin_mem[c_slot][c_tx] <= seg_min_n;
            tmax_mem[c_slot][c_tx] <= seg_max_n;
          end else begin
            tmin_mem[c_slot][c_tx] <= (seg_min_n < acc_min_q) ? seg_min_n : acc_min_q;
            tmax_mem[c_slot][c_tx] <= (seg_max_n > acc_max_q) ? seg_max_n : acc_max_q;
          end
        end
      end
    end
  end

  // --------------------------------------------------------------- output --
  typedef enum logic [1:0] {
    S_IDLE,
    S_LOAD0,
    S_LOAD1,
    S_RUN
  } state_t;
  state_t st;
  logic [11:0] ox;

  // Tile row r used by output row oy, and the input rows it needs.
  wire [9:0] ty_raw = oy[11:2];
  wire [9:0] r = (ty_raw >= tiles_y) ? tiles_y - 1'b1 : ty_raw;
  wire has_up = r != '0;
  wire has_down = (r + 1'b1) < tiles_y;
  wire [11:0] need_stats = {(has_down ? r + 10'd2 : r + 10'd1), 2'b00};
  wire [11:0] need_rows = (oy + 1'b1 > need_stats) ? oy + 1'b1 : need_stats;
  wire row_ready = frame_out && rows_done >= need_rows;

  // Vertical min/max of tile column col_addr over tile rows r-1..r+1 (clipped):
  // one registered read of each slot memory, combined the next cycle.
  logic [TA-1:0] col_addr;
  logic col_neutral_q;  // the column doesn't exist: neutral stats
  logic col_neutral_d;  // ...lined up with the read data
  logic [7:0] rd_min[4], rd_max[4];
  logic [1:0] slot_c, slot_u, slot_d;
  logic use_u, use_d;
  always_ff @(posedge clk) begin
    for (int s = 0; s < 4; s++) begin
      rd_min[s] <= tmin_mem[s][col_addr];
      rd_max[s] <= tmax_mem[s][col_addr];
    end
    col_neutral_d <= col_neutral_q;
    slot_c <= r[1:0];
    slot_u <= r[1:0] - 1'b1;
    slot_d <= r[1:0] + 1'b1;
    use_u <= has_up;
    use_d <= has_down;
  end
  logic [7:0] vmin, vmax;
  always_comb begin
    vmin = rd_min[slot_c];
    vmax = rd_max[slot_c];
    if (use_u && rd_min[slot_u] < vmin) vmin = rd_min[slot_u];
    if (use_u && rd_max[slot_u] > vmax) vmax = rd_max[slot_u];
    if (use_d && rd_min[slot_d] < vmin) vmin = rd_min[slot_d];
    if (use_d && rd_max[slot_d] > vmax) vmax = rd_max[slot_d];
    if (col_neutral_d) begin
      vmin = 8'hFF;
      vmax = 8'h00;
    end
  end

  // Sliding window over tile columns tx-1, tx, tx+1, plus the prefetched tx+2.
  logic [7:0] w_min[3], w_max[3];
  logic [7:0] pend_min, pend_max;
  logic pend_load, pend_load_d;  // prefetch issued / its data on vmin now

  wire [9:0] o_tx = ox[11:2];
  wire in_tiles = o_tx < tiles_x;
  wire new_tile = (ox[1:0] == 2'd0) && (ox != '0) && in_tiles;
  wire [10:0] pre_col = {1'b0, o_tx} + 11'd2;

  logic [7:0] n_min[3], n_max[3];  // the window for the pixel issued this cycle
  always_comb begin
    if (ox == '0) begin
      n_min = '{w_min[0], w_min[1], vmin};
      n_max = '{w_max[0], w_max[1], vmax};
    end else if (new_tile) begin
      n_min = '{w_min[1], w_min[2], pend_min};
      n_max = '{w_max[1], w_max[2], pend_max};
    end else begin
      n_min = w_min;
      n_max = w_max;
    end
  end
  wire [7:0] min01 = n_min[0] < n_min[1] ? n_min[0] : n_min[1];
  wire [7:0] max01 = n_max[0] > n_max[1] ? n_max[0] : n_max[1];
  wire [7:0] win_min = min01 < n_min[2] ? min01 : n_min[2];
  wire [7:0] win_max = max01 > n_max[2] ? max01 : n_max[2];

  // Pixel stage: issued with its window, decided the next cycle.
  logic s_valid, s_full, s_sof, s_eol;
  logic [11:0] s_x, s_y;
  logic [7:0] s_min, s_max, pix_q;

  always_ff @(posedge clk) begin
    if (rst) begin
      st <= S_IDLE;
      frame_out <= 1'b0;
      s_valid <= 1'b0;
      pend_load <= 1'b0;
      pend_load_d <= 1'b0;
      ox <= '0;
      oy <= '0;
      col_addr <= '0;
      col_neutral_q <= 1'b0;
    end else begin
      s_valid <= 1'b0;
      pend_load <= 1'b0;
      pend_load_d <= pend_load;
      if (pend_load_d) begin
        pend_min <= vmin;
        pend_max <= vmax;
      end
      if (i_valid && i_sof) begin
        frame_out <= 1'b1;
        ox <= '0;
        oy <= '0;
        st <= S_IDLE;
      end else begin
        unique case (st)
          S_IDLE:
          if (row_ready) begin
            col_addr <= '0;
            col_neutral_q <= 1'b0;
            st <= S_LOAD0;
          end
          S_LOAD0: begin  // column 0's data arrives next cycle; ask for column 1
            col_addr <= TA'(1);
            col_neutral_q <= tiles_x < 10'd2;
            st <= S_LOAD1;
          end
          S_LOAD1: begin  // column 0 on vmin now; column 1 next cycle
            w_min[0] <= 8'hFF;
            w_max[0] <= 8'h00;
            w_min[1] <= vmin;
            w_max[1] <= vmax;
            st <= S_RUN;
          end
          S_RUN: begin
            if (ox == '0) begin
              w_min[2] <= vmin;  // column 1 (or neutral)
              w_max[2] <= vmax;
            end else if (new_tile) begin
              w_min <= n_min;
              w_max <= n_max;
            end
            // Prefetch column tx+2 on each tile's second pixel.
            if (ox[1:0] == 2'd1 && in_tiles) begin
              col_addr <= TA'(pre_col);
              col_neutral_q <= pre_col >= {1'b0, tiles_x};
              pend_load <= 1'b1;
            end
            s_valid <= 1'b1;
            s_x <= ox;
            s_y <= oy;
            s_sof <= ox == '0 && oy == '0;
            s_eol <= ox == width - 1'b1;
            s_full <= (ox < full_w) && (oy < full_h);
            s_min <= win_min;
            s_max <= win_max;
            pix_q <= pix_mem[pix_addr(ox, oy[3:0])];
            if (ox == width - 1'b1) begin
              ox <= '0;
              if (oy == height - 1'b1) frame_out <= 1'b0;
              else oy <= oy + 1'b1;
              st <= S_IDLE;
            end else begin
              ox <= ox + 1'b1;
            end
          end
        endcase
      end
    end
  end

  // Stage 2: the threshold decision.
  wire [8:0] range = {1'b0, s_max} - {1'b0, s_min};
  wire [7:0] thresh = s_min + range[8:1];
  always_ff @(posedge clk) begin
    if (rst) begin
      o_valid <= 1'b0;
    end else begin
      o_valid <= s_valid;
      o_x <= s_x;
      o_y <= s_y;
      o_sof <= s_sof;
      o_eol <= s_eol;
      if (s_full && range < {1'b0, min_white_black_diff}) o_pix <= 8'd127;
      else o_pix <= (pix_q > thresh) ? 8'd255 : 8'd0;
    end
  end

endmodule
