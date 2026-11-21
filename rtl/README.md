# rtl/

SystemVerilog for the FPGA logic: MIPI capture and the timestamp counter, FrameBus, the streaming AprilTag pipeline, the downscaler for the DPU, and the CAN-FD glue. Vendor IP (MIPI CSI-2 RX, DPU, VCU) is instantiated from the block design in `platform/`, not copied here.

- **Layout:** one folder per block (`rtl/<block>/`), each with its own top module.
- **Style:** `logic`, `always_ff`/`always_comb`, synchronous active-high reset. It must lint clean with Verible and with Verilator `-Wall`.
- **Filled in by:** Stages [3](../docs/stages/stage-3-camera-capture.md) and [4](../docs/stages/stage-4-apriltag-detector.md) onward.
- **CI:** `scripts/lint-rtl.sh` (Verible lint and format check) runs on every `.sv` file here.
