// tag36h11 ID lookup (PLAN step 6), bit-exact with the golden model's
// Tag36h11Lookup::decode() (model/include/gv/model/tag_lookup.h), which
// follows NAPA's nearest_code() and errors_connected():
//   - a read within 3 bits of a code shares one of its four 9-bit blocks
//     exactly, so each block's value indexes a bucket of candidate codes
//     (at most 11), compared by popcount; the first within max_bits wins
//     (codes are >= 11 apart, so it is the only one);
//   - a 3-bit correction is kept only if the wrong cells form one 8-connected
//     group.
// Searching blocks 0..3 in order, at most 4 x 11 compares: under 50 cycles.
//
// The tables are ROMs from `gv-model lookup-tables DIR`, loaded with
// $readmemh from TABLE_DIR, or in simulation from +LOOKUP_TABLES=DIR.
// This version reads them asynchronously; block RAM needs a registered read,
// which adds a cycle per compare.
module id_lookup #(
    parameter string TABLE_DIR = ""
) (
    input logic clk,
    input logic rst,
    input logic in_valid,
    output logic in_ready,
    input logic [35:0] in_code,
    input logic [1:0] max_bits,  // 0..3
    output logic out_valid,
    output logic out_found,
    output logic [9:0] out_id,
    output logic [1:0] out_rotation,
    output logic [1:0] out_hamming
);

  localparam int NROT   = 2348;  // 587 codes x 4 rotations
  localparam int NCODES = 587;

  logic [35:0] codes[NROT];
  logic [35:0] near_mask[36];  // near_mask[k]: cells touching cell k (bit 35 - k is cell k)
  // One ROM per 9-bit block ($readmemh can't load a slice of a 2-D array).
  logic [15:0] start0[513], start1[513], start2[513], start3[513];
  logic [11:0] list0[NROT], list1[NROT], list2[NROT], list3[NROT];

`ifdef SYNTHESIS
  initial begin
    $readmemh({TABLE_DIR, "/codes.mem"}, codes);
    $readmemh({TABLE_DIR, "/near.mem"}, near_mask);
    $readmemh({TABLE_DIR, "/start_0.mem"}, start0);
    $readmemh({TABLE_DIR, "/start_1.mem"}, start1);
    $readmemh({TABLE_DIR, "/start_2.mem"}, start2);
    $readmemh({TABLE_DIR, "/start_3.mem"}, start3);
    $readmemh({TABLE_DIR, "/list_0.mem"}, list0);
    $readmemh({TABLE_DIR, "/list_1.mem"}, list1);
    $readmemh({TABLE_DIR, "/list_2.mem"}, list2);
    $readmemh({TABLE_DIR, "/list_3.mem"}, list3);
  end
`else
  initial begin
    string dir;
    dir = TABLE_DIR;
    if ($value$plusargs("LOOKUP_TABLES=%s", dir)) begin
    end
    $readmemh({dir, "/codes.mem"}, codes);
    $readmemh({dir, "/near.mem"}, near_mask);
    $readmemh({dir, "/start_0.mem"}, start0);
    $readmemh({dir, "/start_1.mem"}, start1);
    $readmemh({dir, "/start_2.mem"}, start2);
    $readmemh({dir, "/start_3.mem"}, start3);
    $readmemh({dir, "/list_0.mem"}, list0);
    $readmemh({dir, "/list_1.mem"}, list1);
    $readmemh({dir, "/list_2.mem"}, list2);
    $readmemh({dir, "/list_3.mem"}, list3);
  end
`endif

  function automatic logic [5:0] popcount36(input logic [35:0] v);
    logic [5:0] c;
    c = '0;
    for (int b = 0; b < 36; b++) c = c + 6'(v[b]);
    return c;
  endfunction

  // Whether the set bits of diff (at most 3 here) form one 8-connected group.
  function automatic logic connected(input logic [35:0] diff);
    logic [35:0] reached, grow;
    reached = diff & (~diff + 1'b1);  // the lowest set bit
    for (int step = 0; step < 3; step++) begin
      grow = reached;
      for (int p = 0; p < 36; p++) if (reached[p]) grow = grow | (near_mask[35-p] & diff);
      reached = grow;
    end
    return reached == diff;
  endfunction

  typedef enum logic [1:0] {
    S_IDLE,
    S_BUCKET,
    S_SCAN,
    S_DONE
  } state_t;
  state_t st;

  logic [35:0] code;
  logic [1:0] limit;
  logic [1:0] blk;
  logic [15:0] e, e_end;

  wire [9:0] block_value = {1'b0, code[9*blk+:9]};
  function automatic logic [15:0] start_at(input logic [1:0] b, input logic [9:0] v);
    unique case (b)
      2'd0: return start0[v];
      2'd1: return start1[v];
      2'd2: return start2[v];
      default: return start3[v];
    endcase
  endfunction
  function automatic logic [11:0] list_at(input logic [1:0] b, input logic [11:0] i);
    unique case (b)
      2'd0: return list0[i];
      2'd1: return list1[i];
      2'd2: return list2[i];
      default: return list3[i];
    endcase
  endfunction
  wire [11:0] idx = list_at(blk, e[11:0]);
  wire [35:0] diff = code ^ codes[idx];
  wire [ 5:0] hamming_d = popcount36(diff);

  assign in_ready = st == S_IDLE;

  always_ff @(posedge clk) begin
    out_valid <= 1'b0;
    if (rst) begin
      st <= S_IDLE;
    end else begin
      unique case (st)
        S_IDLE:
        if (in_valid) begin
          code <= in_code;
          limit <= max_bits;
          blk <= '0;
          st <= S_BUCKET;
        end
        S_BUCKET: begin
          e <= start_at(blk, block_value);
          e_end <= start_at(blk, block_value + 1'b1);
          st <= S_SCAN;
        end
        S_SCAN: begin
          if (e < e_end) begin
            if (hamming_d <= 6'(limit)) begin
              // The only code this near. A 3-bit fix must be one touching group.
              out_valid <= 1'b1;
              out_found <= hamming_d != 6'd3 || connected(diff);
              out_hamming <= hamming_d[1:0];
              out_rotation <= idx >= 12'(3 * NCODES) ? 2'd3 : idx >= 12'(2 * NCODES) ? 2'd2 :
                  idx >= 12'(NCODES) ? 2'd1 : 2'd0;
              out_id <= 10'(idx >= 12'(3 * NCODES) ? idx - 12'(3 * NCODES) :
                            idx >= 12'(2 * NCODES) ? idx - 12'(2 * NCODES) :
                            idx >= 12'(NCODES) ? idx - 12'(NCODES) : idx);
              st <= S_DONE;
            end else begin
              e <= e + 1'b1;
            end
          end else if (blk == 2'd3) begin
            out_valid <= 1'b1;
            out_found <= 1'b0;
            st <= S_DONE;
          end else begin
            blk <= blk + 1'b1;
            st  <= S_BUCKET;
          end
        end
        S_DONE:  st <= S_IDLE;
        default: st <= S_IDLE;
      endcase
    end
  end

endmodule
