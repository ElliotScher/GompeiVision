# sim/

Verilator + cocotb testbenches. Each one compares an `rtl/` block with its `model/` golden model on synthetic inputs and recorded frames.

- **Layout:** `sim/<block>/test_<block>.py` is a pytest file that builds the block with cocotb's Python runner and runs the cocotb tests next to it. [`tools/selftest/`](../tools/selftest/) shows the pattern.
- **Run:** `scripts/run-sims.sh` runs every testbench plus the toolchain self-test, the same way CI does.
- **Filled in by:** Stage [3](../docs/stages/stage-3-camera-capture.md) onward.
