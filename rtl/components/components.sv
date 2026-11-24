// Streaming connected components (PLAN step 3), second version. Same
// partition, accumulators and closure order as the golden model
// model/include/gv/model/components.h (and so AprilTag's single-threaded
// connected_components()):
//   - joins exactly as reference_joins(): left, up, up-left and up-right
//     with AprilTag's redundancy shortcuts, which matter at the image edges;
//     127 never joins; black is 4-connected and white 8-connected;
//   - one record per component: first-pixel raster id, color, area, bounding
//     box, and the extreme pixel in each of 16 directions (ties to the
//     earlier pixel in raster order);
//   - a component is emitted after the first row that adds no pixel to it,
//     in left-to-right order of its runs in the row above; components still
//     open after the last row are emitted then, left to right.
//
// Three stages, all working at once, in order throughout:
//   A. Input, one pixel per clock plus 2 cycles per row. Pixels form runs
//      (maximal left-joined stretches of one row); a run's joins to the
//      previous row's runs queue as "pairs". Out come two FIFOs: items (a
//      run's end with up to 2 pairs; more pairs go in earlier items) and the
//      previous row's runs nothing joined ("childless").
//   B. Label resolver. Labels only: a run takes its first pair's root, further
//      roots merge with it, and a run without pairs gets a new label. After a
//      row, each childless run whose root the row didn't touch closes it. One
//      cycle per pair or pairless run, one more per run with several roots,
//      one per childless run, plus a cycle per union-find hop.
//   C. Record engine, one operation per clock: create, add a run, merge,
//      emit. Two copies of the record memory (two reads, one write), reads
//      registered as block RAM does, and the last write forwarded.
//
// Labels are recycled. A label merged away or closed in row y can still be
// named by rows y - 1 and y, whose last use is row y + 1, so it is free after
// that. Every label in use is named by one of 3 rows, so 3 * MAX_W are enough
// for any image; o_overflow (sticky) is set if they ever run out.
//
// A start of frame always restarts, even mid-row. o_frame_start comes before a
// frame's first record and o_frame_done after its last; both can trail the
// input by however much is queued. A frame cut short has no o_frame_done.
module components #(
    parameter int MAX_W = 1280,
    parameter int MAX_LABELS = 4096,
    parameter int ITEM_DEPTH = 2048
) (
    input logic clk,
    input logic rst,
    input logic [11:0] i_width,
    input logic [11:0] i_height,
    input logic in_valid,
    output logic in_ready,
    input logic in_sof,
    input logic [7:0] in_pix,  // 0, 127 or 255

    output logic        o_valid,
    output logic [31:0] o_id,
    output logic        o_white,
    output logic [31:0] o_area,
    output logic [11:0] o_xmin,
    output logic [11:0] o_xmax,
    output logic [11:0] o_ymin,
    output logic [11:0] o_ymax,
    output logic [11:0] o_closed_after_row,
    output logic [11:0] o_ex              [16],
    output logic [11:0] o_ey              [16],
    output logic        o_frame_start,
    output logic        o_frame_done,            // after the frame's last record
    output logic        o_overflow,
    output logic [31:0] o_row_cycles_max,        // most resolver cycles for one row
    output logic [31:0] o_labels_max             // most labels ever allocated at once
);

  localparam int LW = $clog2(MAX_LABELS);
  localparam int XI = $clog2(MAX_W);  // column and run index bits
  localparam int FA = $clog2(ITEM_DEPTH);
  localparam logic [1:0] Skip = 2'd2, Out = 2'd3;  // pixel codes; 0 black, 1 white

  // Direction vectors, round(64 * (cos, sin)) of k * 22.5 degrees (as kDirectionVectors).
  localparam int DX[16] = '{64, 59, 45, 24, 0, -24, -45, -59, -64, -59, -45, -24, 0, 24, 45, 59};
  localparam int DY[16] = '{0, 24, 45, 59, 64, 59, 45, 24, 0, -24, -45, -59, -64, -59, -45, -24};

  // =============================================================== stage A --
  typedef enum logic [1:0] {
    A_IN,
    A_END1,
    A_END2
  } astate_t;
  astate_t ast;

  // Item kinds: RUN is the end of a run, PAIRS more pairs of an unfinished run.
  typedef enum logic [1:0] {
    I_RUN,
    I_PAIRS,
    I_ROW,
    I_SOF
  } kind_t;
  typedef struct packed {
    kind_t         kind;
    logic          white;      // I_ROW: the frame's last row
    logic [11:0]   xmin;       // I_SOF: width
    logic [11:0]   xmax;
    logic [1:0]    np;
    logic [XI-1:0] pa,    pb;
  } item_t;
  item_t it_mem[ITEM_DEPTH];
  logic [FA:0] it_w, it_r;
  // Childless runs: {marker, run}. A marker ends each row, and starts a frame
  // (with run = 1), so the resolver can skip a cut row's leftovers.
  logic [XI:0] cl_mem[ITEM_DEPTH];
  logic [FA:0] cl_w, cl_r;
  wire room = (it_w - it_r) < (FA + 1)'(ITEM_DEPTH - 4) &&
      (cl_w - cl_r) < (FA + 1)'(ITEM_DEPTH - 4);

  // The last MAX_W pixels, {code, run index}: column c holds the previous
  // row's value until the current row's replaces it.
  logic [XI+1:0] row_mem[MAX_W];
  logic [XI+1:0] w_ul, w_up;  // the previous row at x-1 and x
  logic a_active;
  logic [11:0] ax, ay, aw, ah;

  // The open run and its waiting pairs.
  logic run_open, run_white;
  logic [1:0] left;  // the left pixel's code
  logic [11:0] run_xmin, run_xmax;
  logic [XI-1:0] run_idx;  // the open run's, or the next one's
  logic [1:0] np;
  logic [XI-1:0] pa, pb;
  logic [3:0] joined;  // previous-row runs (index mod 4) joined so far: at most 3 matter at once

  wire [11:0] px = in_sof ? 12'd0 : ax;
  wire [11:0] py = in_sof ? 12'd0 : ay;
  wire [11:0] pw = in_sof ? i_width : aw;
  wire [XI+1:0] w_ur = row_mem[XI'(px+12'd1)];
  wire take = in_valid && in_ready && (in_sof || a_active);
  assign in_ready = ast == A_IN && room;

  logic [1:0] v, c_left, c_ul, c_up, c_ur;
  logic is_ref, j_left, j_up, j_ul, j_ur;
  logic [XI-1:0] r_ul, r_up, r_ur;
  always_comb begin
    v = in_pix == 8'd0 ? 2'd0 : (in_pix == 8'd255 ? 2'd1 : Skip);
    r_ul = w_ul[XI-1:0];
    r_up = w_up[XI-1:0];
    r_ur = w_ur[XI-1:0];
    c_left = px != 0 ? left : Out;
    c_ul = py != 0 && px != 0 ? w_ul[XI+1:XI] : Out;
    c_up = py != 0 ? w_up[XI+1:XI] : Out;
    c_ur = py != 0 && px + 12'd1 < pw ? w_ur[XI+1:XI] : Out;
    is_ref = v != Skip && px >= 12'd1 && px + 12'd2 <= pw;
    j_left = is_ref && c_left == v;
    j_up = is_ref && py != 0 && (px == 12'd1 || !(c_left == c_ul && c_ul == c_up)) && c_up == v;
    // With j_up too (column 1 only), up-left is the same run as up.
    j_ul = is_ref && py != 0 && v == 2'd1 && !j_up &&
        (px == 12'd1 || !(c_left == c_ul || c_up == c_ul)) && c_ul == v;
    j_ur = is_ref && py != 0 && v == 2'd1 && c_up != c_ur && c_ur == v;
  end

  // This pixel's pairs, at most 2: up-left and up-right. Then it starts a run
  // (up-left needs a left pixel of the other color), except in column 1.
  wire [1:0] new_n = 2'(j_up) + 2'(j_ul) + 2'(j_ur);
  wire [XI-1:0] new_a = j_up ? r_up : (j_ul ? r_ul : r_ur);
  logic [3:0] joined_n;
  always_comb begin
    joined_n = joined;
    if (j_up) joined_n[r_up[1:0]] = 1'b1;
    if (j_ul) joined_n[r_ul[1:0]] = 1'b1;
    if (j_ur) joined_n[r_ur[1:0]] = 1'b1;
  end
  // A previous-row run ending at x-1 can't be joined after column x.
  wire ul_ends = c_ul < Skip && (c_up >= Skip || r_up != r_ul);

  wire [XI-1:0] next_idx = in_sof ? '0 : run_idx + XI'(run_open);

  logic it_push, cl_push;
  item_t it_new;
  logic [XI:0] cl_new;
  always_comb begin
    it_push = 1'b0;
    cl_push = 1'b0;
    it_new = '{
        kind: I_RUN,
        white: run_white,
        xmin: run_xmin,
        xmax: run_xmax,
        np: np,
        pa: pa,
        pb: pb
    };
    cl_new = {1'b0, r_ul};
    if (take) begin
      if (in_sof) begin
        it_push = 1'b1;
        it_new.kind = I_SOF;
        it_new.xmin = i_width;
        cl_push = 1'b1;
        cl_new = {1'b1, XI'(1)};
      end else begin
        if (run_open && !j_left) it_push = 1'b1;
        else if (j_left && new_n != 0 && np == 2'd2) begin
          it_push = 1'b1;
          it_new.kind = I_PAIRS;
        end
        cl_push = ul_ends && !joined_n[r_ul[1:0]];
      end
    end else if (ast == A_END1) begin
      it_push = run_open;
      cl_push = ay != 0 && w_ul[XI+1:XI] < Skip && !joined[r_ul[1:0]];
    end else if (ast == A_END2) begin
      it_push = 1'b1;
      it_new.kind = I_ROW;
      it_new.white = ay == ah - 12'd1;
      cl_push = 1'b1;
      cl_new = {1'b1, XI'(0)};
    end
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      ast <= A_IN;
      a_active <= 1'b0;
      it_w <= '0;
      cl_w <= '0;
    end else begin
      if (it_push) begin
        it_mem[it_w[FA-1:0]] <= it_new;
        it_w <= it_w + 1'b1;
      end
      if (cl_push) begin
        cl_mem[cl_w[FA-1:0]] <= cl_new;
        cl_w <= cl_w + 1'b1;
      end
      if (take) begin
        if (in_sof) begin
          a_active <= 1'b1;
          aw <= i_width;
          ah <= i_height;
        end
        ay <= py;
        ax <= px + 12'd1;
        left <= v;
        w_ul <= w_up;
        w_up <= w_ur;
        // A joined flag goes back to 0 when its run has been checked, so it's
        // clear when the index mod 4 comes around again.
        joined <= in_sof ? '0 : joined_n;
        if (!in_sof && ul_ends) joined[r_ul[1:0]] <= 1'b0;
        row_mem[XI'(px)] <= {v, j_left ? run_idx : next_idx};
        if (j_left) begin
          run_xmax <= px;
          if (new_n == 2'd2) begin
            np <= 2'd2;  // column 1: the run opened at column 0, so it has no pairs yet
            pa <= new_a;
            pb <= r_ur;
          end else if (new_n != 0) begin
            np <= np == 2'd2 ? 2'd1 : np + 2'd1;
            if (np == 2'd1) pb <= new_a;
            else pa <= new_a;
          end
        end else begin
          run_open <= v != Skip;
          run_idx <= next_idx;
          run_white <= v == 2'd1;
          run_xmin <= px;
          run_xmax <= px;
          np <= new_n;
          pa <= new_a;
          pb <= r_ur;
        end
        if (px == pw - 12'd1) ast <= A_END1;
      end else if (ast == A_END1) begin
        joined <= '0;
        run_open <= 1'b0;
        run_idx <= '0;
        ast <= A_END2;
      end else if (ast == A_END2) begin
        w_up <= row_mem[0];
        ax   <= '0;
        ay   <= ay + 12'd1;
        if (ay == ah - 12'd1) a_active <= 1'b0;
        ast <= A_IN;
      end
    end
  end

  // =============================================================== stage B --
  typedef enum logic [2:0] {
    B_ITEM,
    B_CLOSE,
    B_FINAL,
    B_DONE,
    B_FLUSH
  } bstate_t;
  bstate_t bst;

  logic [LW-1:0] label_mem[2][MAX_W];  // by row parity and run
  logic [LW-1:0] parent[MAX_LABELS];
  // Per label: the parity of the last row that touched it (a live component
  // was touched in this row or the last), and whether it was emitted.
  logic touched[MAX_LABELS];
  logic emitted[MAX_LABELS];
  // Runs in the component (saturating). The bigger root survives a merge, so
  // labels already handed out mostly stay roots: keeping the left one instead
  // costs about 400 hop cycles per row on noise at 1280 wide, this about 35.
  localparam int SW = 16;
  logic [SW-1:0] size[MAX_LABELS];

  // Free labels: a ring of retired ones, usable up to fr_ok, and never-used
  // ones from `fresh` up.
  logic [LW-1:0] free_mem[MAX_LABELS];
  logic [LW:0] fr_w, fr_r, fr_ok, fr_mark;
  logic [LW:0] fresh;

  wire item_t item = it_mem[it_r[FA-1:0]];
  wire it_have = it_w != it_r;
  wire [XI:0] child = cl_mem[cl_r[FA-1:0]];
  wire cl_have = cl_w != cl_r;

  logic [11:0] by, bw;
  logic [31:0] base;  // by * bw
  logic bank;  // the current row's label bank
  logic [XI-1:0] bidx;  // current row's run counter, and B_FINAL's index
  logic [XI:0] n_runs;
  logic have_r;  // the open run has a root
  logic [LW-1:0] root_r;
  logic [SW-1:0] size_r;  // size[root_r]
  logic [1:0] step;  // which of the item's pairs, or 2: add the run
  logic final_q;  // bidx scans the last row

  // Find: one hop per cycle from the node a run names.
  logic hopping, hopped2;
  logic [LW-1:0] node, first;
  logic [XI-1:0] find_run;
  logic find_cur;
  always_comb begin
    find_cur = 1'b0;
    find_run = step[0] ? item.pb : item.pa;
    if (bst == B_CLOSE) find_run = child[XI-1:0];
    else if (bst == B_FINAL) begin
      find_run = bidx;
      find_cur = 1'b1;
    end
  end
  wire [LW-1:0] n0 = hopping ? node : label_mem[find_cur ? bank : ~bank][find_run];
  wire [LW-1:0] n1 = parent[n0];
  wire at_root = n1 == n0;

  wire use_ring = fr_r != fr_ok;
  wire [LW-1:0] new_label = use_ring ? free_mem[fr_r[LW-1:0]] : fresh[LW-1:0];
  wire no_label = !use_ring && fresh == (LW + 1)'(MAX_LABELS);

  wire [SW-1:0] size_n0 = size[n0];
  wire [SW:0] size_sum = (SW + 1)'(size_n0) + (SW + 1)'(size_r);
  wire [SW-1:0] size_merged = size_sum[SW] ? '1 : size_sum[SW-1:0];
  wire n0_wins = size_n0 > size_r;
  wire [LW-1:0] loser = n0_wins ? root_r : n0;

  // Record engine operations.
  typedef enum logic [2:0] {
    OP_NONE,
    OP_NEW,
    OP_ADD,
    OP_MERGE,
    OP_EMIT,
    OP_START,
    OP_DONE
  } op_t;
  op_t op;
  logic [LW-1:0] op_a, op_b;

  // This cycle's decisions.
  logic finding, want_merge, run_done, do_new, do_retire, do_emit;
  logic [LW-1:0] run_root, retire_label;
  logic [SW-1:0] run_size;  // run_root's, before this run
  logic it_pop, cl_pop;
  always_comb begin
    finding = 1'b0;
    want_merge = 1'b0;
    run_done = 1'b0;
    do_new = 1'b0;
    do_emit = 1'b0;
    run_root = root_r;
    run_size = size_r;
    it_pop = 1'b0;
    cl_pop = 1'b0;
    op = OP_NONE;
    op_a = root_r;
    op_b = n0;
    unique case (bst)
      B_ITEM:
      if (it_have) begin
        if (item.kind == I_SOF) begin
          op = OP_START;
          it_pop = 1'b1;
        end else if (item.kind == I_ROW) begin
          it_pop = 1'b1;
        end else if (step == 2'd2) begin
          op = OP_ADD;
          run_done = 1'b1;
          it_pop = 1'b1;
        end else if (item.np == 0) begin
          op = OP_NEW;
          op_a = new_label;
          run_root = new_label;
          run_size = '0;
          do_new = 1'b1;
          run_done = 1'b1;
          it_pop = 1'b1;
        end else begin
          finding = 1'b1;
          if (at_root) begin
            want_merge = have_r && n0 != root_r;
            if (!have_r || (want_merge && n0_wins)) run_root = n0;
            if (!have_r) run_size = size_n0;
            if (want_merge) begin
              run_size = size_merged;
              op = OP_MERGE;
              op_a = run_root;
              op_b = loser;
            end
            if (2'(step) + 2'd1 == item.np) begin
              if (item.kind == I_PAIRS) it_pop = 1'b1;
              else if (!want_merge) begin
                op = OP_ADD;
                op_a = run_root;
                run_done = 1'b1;
                it_pop = 1'b1;
              end
            end
          end
        end
      end
      B_CLOSE:
      if (cl_have) begin
        if (child[XI]) cl_pop = 1'b1;
        else begin
          finding = 1'b1;
          if (at_root) begin
            cl_pop  = 1'b1;
            do_emit = !emitted[n0] && touched[n0] != by[0];
          end
        end
      end
      B_FINAL:
      if ((XI + 1)'(bidx) < n_runs) begin
        finding = 1'b1;
        do_emit = at_root && !emitted[n0];
      end
      B_DONE:  op = OP_DONE;
      B_FLUSH: cl_pop = cl_have;
      default: ;
    endcase
    if (do_emit) begin
      op   = OP_EMIT;
      op_a = n0;
    end
    do_retire = want_merge || do_emit;
    retire_label = do_emit ? n0 : loser;
  end

  logic [31:0] row_cycles;
  wire busy = bst == B_ITEM ? it_have : 1'b1;

  always_ff @(posedge clk) begin
    if (rst) begin
      bst <= B_ITEM;
      it_r <= '0;
      cl_r <= '0;
      have_r <= 1'b0;
      step <= '0;
      hopping <= 1'b0;
      o_overflow <= 1'b0;
      o_row_cycles_max <= '0;
      o_labels_max <= '0;
      row_cycles <= '0;
      fr_w <= '0;
      fr_r <= '0;
      fr_ok <= '0;
      fr_mark <= '0;
      fresh <= '0;
    end else begin
      if (it_pop) it_r <= it_r + 1'b1;
      if (cl_pop) cl_r <= cl_r + 1'b1;
      if (busy) row_cycles <= row_cycles + 32'd1;

      // Union-find. One parent write per cycle: a merge first, else a new
      // label, else path compression after 2 or more hops.
      if (finding) begin
        hopping <= !at_root;
        hopped2 <= hopping && !at_root;
        if (!at_root) node <= n1;
        if (!hopping) first <= n0;
      end
      if (want_merge) parent[loser] <= run_root;
      else if (do_new) parent[new_label] <= new_label;
      else if (finding && at_root && hopped2) parent[first] <= n0;

      if (want_merge) size[run_root] <= size_merged;
      else if (run_done) size[run_root] <= run_size + SW'(run_size != '1);

      if (do_retire) begin
        free_mem[fr_w[LW-1:0]] <= retire_label;
        fr_w <= fr_w + 1'b1;
      end
      if (do_emit) emitted[n0] <= 1'b1;

      if (bst == B_ITEM && it_have) begin
        if (item.kind == I_SOF) begin
          bw <= item.xmin;
          by <= '0;
          base <= '0;
          bank <= 1'b0;
          bidx <= '0;
          have_r <= 1'b0;
          step <= '0;
          fr_w <= '0;
          fr_r <= '0;
          fr_ok <= '0;
          fr_mark <= '0;
          fresh <= '0;
          row_cycles <= '0;
          bst <= B_FLUSH;
        end else if (item.kind == I_ROW) begin
          n_runs <= (XI + 1)'(bidx);
          final_q <= item.white;
          bst <= B_CLOSE;
        end else if (finding && at_root) begin
          root_r <= run_root;
          size_r <= run_size;
          have_r <= 1'b1;
          step   <= 2'(step) + 2'd1 == item.np && item.kind == I_RUN ? 2'd2 : step + 2'd1;
          if (it_pop) step <= '0;
        end
        if (run_done) begin
          label_mem[bank][bidx] <= run_root;
          touched[run_root] <= by[0];
          bidx <= bidx + 1'b1;
          have_r <= 1'b0;
          step <= '0;
        end
        if (do_new) begin
          emitted[new_label] <= 1'b0;
          if (no_label) o_overflow <= 1'b1;
          else if (use_ring) fr_r <= fr_r + 1'b1;
          else begin
            fresh <= fresh + 1'b1;
            if (32'(fresh) + 32'd1 > o_labels_max) o_labels_max <= 32'(fresh) + 32'd1;
          end
        end
      end

      if (bst == B_CLOSE && cl_have && child[XI]) begin
        bidx <= '0;
        if (row_cycles > o_row_cycles_max) o_row_cycles_max <= row_cycles;
        row_cycles <= '0;
        if (final_q) bst <= B_FINAL;
        else begin
          bank <= ~bank;
          by <= by + 12'd1;
          base <= base + 32'(bw);
          fr_ok <= fr_mark;
          fr_mark <= fr_w;
          bst <= B_ITEM;
        end
      end
      if (bst == B_FINAL) begin
        if ((XI + 1)'(bidx) < n_runs) begin
          if (at_root) bidx <= bidx + 1'b1;
        end else begin
          bst <= B_DONE;
        end
      end
      if (bst == B_DONE) bst <= B_ITEM;
      if (bst == B_FLUSH && cl_have && child == {1'b1, XI'(1)}) bst <= B_ITEM;
    end
  end

  // =============================================================== stage C --
  typedef struct packed {
    logic [31:0]       id;
    logic              white;
    logic [31:0]       area;
    logic [11:0]       xmin,  xmax, ymin, ymax;
    logic [15:0][11:0] ex,    ey;
  } rec_t;
  // The same records twice, for two reads. 4,096 is UltraRAM's depth.
  (* ram_style = "ultra" *)rec_t mem_a[MAX_LABELS];
  (* ram_style = "ultra" *)rec_t mem_b[MAX_LABELS];

  // Is (bx, by) the better extreme for direction k? Larger score, ties to the
  // earlier pixel in raster order.
  function automatic logic better(input logic [3:0] k, input logic [11:0] ux, input logic [11:0] uy,
                                  input logic [11:0] vx, input logic [11:0] vy);
    logic signed [31:0] s;
    s = DX[k] * (32'(vx) - 32'(ux)) + DY[k] * (32'(vy) - 32'(uy));
    return s > 0 || (s == 0 && {vy, vx} < {uy, ux});
  endfunction

  // Register the operation, then read both records.
  op_t op1, op2;
  logic [LW-1:0] a1, b1, a2, b2;
  rec_t run1, run2, ram_a, ram_b;
  logic [11:0] row1, row2;
  // The last write, which a read in the same cycle missed.
  logic fw;
  logic [LW-1:0] fw_addr;
  rec_t fw_data;

  rec_t old_a, old_b, rhs, merged;
  logic we;
  always_comb begin
    old_a = fw && fw_addr == a2 ? fw_data : ram_a;
    old_b = fw && fw_addr == b2 ? fw_data : ram_b;
    rhs = op2 == OP_MERGE ? old_b : run2;
    merged = old_a;
    if (rhs.id < old_a.id) merged.id = rhs.id;
    merged.area = old_a.area + rhs.area;
    if (rhs.xmin < old_a.xmin) merged.xmin = rhs.xmin;
    if (rhs.xmax > old_a.xmax) merged.xmax = rhs.xmax;
    if (rhs.ymin < old_a.ymin) merged.ymin = rhs.ymin;
    if (rhs.ymax > old_a.ymax) merged.ymax = rhs.ymax;
    for (int k = 0; k < 16; k++) begin
      if (better(4'(k), old_a.ex[k], old_a.ey[k], rhs.ex[k], rhs.ey[k])) begin
        merged.ex[k] = rhs.ex[k];
        merged.ey[k] = rhs.ey[k];
      end
    end
    if (op2 == OP_NEW) merged = run2;
    we = op2 == OP_NEW || op2 == OP_ADD || op2 == OP_MERGE;
  end

  always_ff @(posedge clk) begin
    // The run as a record: every pixel has the same y, so the right end wins
    // for dx > 0, else the left end (dx == 0 ties go to the earlier pixel).
    op1 <= rst ? OP_NONE : op;
    a1 <= op_a;
    b1 <= op_b;
    row1 <= by;
    run1.id <= base + 32'(item.xmin);
    run1.white <= item.white;
    run1.area <= 32'(item.xmax - item.xmin) + 32'd1;
    run1.xmin <= item.xmin;
    run1.xmax <= item.xmax;
    run1.ymin <= by;
    run1.ymax <= by;
    for (int k = 0; k < 16; k++) begin
      run1.ex[k] <= DX[k] > 0 ? item.xmax : item.xmin;
      run1.ey[k] <= by;
    end

    op2 <= rst ? OP_NONE : op1;
    a2 <= a1;
    b2 <= b1;
    run2 <= run1;
    row2 <= row1;
    ram_a <= mem_a[a1];
    ram_b <= mem_b[b1];

    fw <= we && !rst;
    fw_addr <= a2;
    fw_data <= merged;
    if (we) begin
      mem_a[a2] <= merged;
      mem_b[a2] <= merged;
    end

    o_valid <= !rst && op2 == OP_EMIT;
    o_frame_start <= !rst && op2 == OP_START;
    o_frame_done <= !rst && op2 == OP_DONE;
    if (op2 == OP_EMIT) begin
      o_id <= old_a.id;
      o_white <= old_a.white;
      o_area <= old_a.area;
      o_xmin <= old_a.xmin;
      o_xmax <= old_a.xmax;
      o_ymin <= old_a.ymin;
      o_ymax <= old_a.ymax;
      o_closed_after_row <= row2;
      for (int k = 0; k < 16; k++) begin
        o_ex[k] <= old_a.ex[k];
        o_ey[k] <= old_a.ey[k];
      end
    end
  end

endmodule
