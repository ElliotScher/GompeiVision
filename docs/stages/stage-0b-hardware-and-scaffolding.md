# Stage 0b: Hardware confirmation, cost check, scaffolding, simulation CI

## Goal

Confirm the SoC FPGA module, dev kit, sensors and per-module cost with real numbers, and set up the repo layout and CI so every later stage has a place to put code and tests.

## Deliverables

- **Hardware report** in this doc: the Kria K26 dev kits (KV260, KR260) against the [selection criteria](../PLAN.md#selection-criteria-stage-0b): MIPI camera ports, logic/DSP/on-chip RAM, DPU support in the current Vitis AI release, free-tier toolchain coverage, price, availability.
- A first resource estimate per PL block (capture, AprilTag pipeline, downscaler, DPU, CAN-FD, infrastructure) against the K26, with ≥30% headroom, and the largest DPU configuration that fits next to the tag pipeline (see [PLAN.md](../PLAN.md#per-module-resource-estimate)).
- On-chip RAM plan: line buffers, label tables, feature accumulators, the decode strip buffer (UltraRAM) and the DPU, against the part's block RAM and UltraRAM.
- CAN-FD check: SystemCore's CAN-FD bus count and bitrates, the WPILib API for custom FD frames (receive queue or newest-per-ID), and the CAN-FD controller (PL IP and its license, or MCP2518FD over SPI) and transceiver.
- Sensor check: OV9281 and OV9782 boards (and AR0234 as the upgrade), whether one carrier connector and one driver can take either, trigger and strobe pins, and a few M12 lens focal lengths tested for tag range and field of view.
- **Cost report:** real quotes for every line of the [module cost table](../PLAN.md#module-cost) (SOM, camera board and lens, carrier board fabrication and assembly, SSD, housing stock, connectors), compared with the ~$400–580 target.
- Module-size check: the K26 footprint against a Limelight-sized housing.
- Recording check: VCU support in the current tools, and an M.2 or USB SSD on the dev kit.
- Whether a newer SoC (e.g. AMD Versal AI Edge Gen 2) is available in a small module yet.
- Purchase list: dev kit, OV9281 and OV9782 camera boards (two of each, to test multiple modules), M12 lenses, cables/adapters, a CAN-FD transceiver breakout, an M.2 2230 NVMe SSD.
- Repo scaffolding: `rtl/`, `sim/`, `platform/`, `rpu/`, `apu/`, `robot/`, `ml/`, `model/`, `tools/tagslam/`, `hardware/`, with a README stub in each.
- GitHub Actions: Verible lint, Verilator + cocotb sims, `model/`, `apu/` and `robot/` unit tests. Bitstream builds on a nightly or self-hosted job.

## Exit criteria

- [ ] Module, dev kit and sensors chosen, with the report filled in and all numbers checked against datasheets.
- [ ] Confirmed that the current Vitis AI release supports a DPU on the K26 (or the report names the object-detection fallback).
- [ ] Per-module cost estimate from real quotes recorded, and within the target or with the gap explained.
- [ ] CI runs on every PR and is green.
- [ ] A "blinky" bitstream loads from Linux on the dev kit.

## Risks

- Vitis AI's focus has been shifting to newer AMD chips; check its support for Zynq UltraScale+ before buying.
- SOM prices change; the cost report records the date of each quote.
