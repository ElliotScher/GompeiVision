// Toolchain self-test: an 8-bit counter with synchronous reset and enable.
// It only checks that Verilator + cocotb build and run a SystemVerilog design,
// so keep it trivial. Run with: scripts/doctor.sh --selftest
module counter #(
    parameter int WIDTH = 8
) (
    input  logic             clk,
    input  logic             rst,
    input  logic             en,
    output logic [WIDTH-1:0] count
);

  always_ff @(posedge clk) begin
    if (rst) count <= '0;
    else if (en) count <= count + 1'b1;
  end

endmodule
