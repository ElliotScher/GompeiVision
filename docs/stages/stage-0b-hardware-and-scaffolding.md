# Stage 0b: Hardware confirmation, scaffolding, simulation CI

## Goal

Confirm the SoC FPGA module, dev kit and sensors with real numbers, and set up the repo layout and CI so every later stage has a place to put code and tests.

## Deliverables

- **Hardware report** in this doc: the Kria K26 dev kits (KV260, KR260) and any other candidates against the [selection criteria](../PLAN.md#selection-criteria-stage-0b): MIPI camera ports, logic/DSP/on-chip RAM, DPU support in the current Vitis AI release, free-tier toolchain coverage, price, availability.
- A first resource estimate per PL block (AprilTag pipeline, VIO feature tracking, DPU on the color variant, capture per camera, CAN-FD) against the chosen part, with ≥30% headroom.
- On-chip RAM plan: line buffers, label tables, feature accumulators, the decode strip buffer (UltraRAM) and the DPU, against the part's block RAM and UltraRAM.
- Fast-path hardware check: SystemCore's CAN-FD bus count and bitrates, the WPILib API for custom FD frames, and the CAN-FD controller (PL IP and its license, or MCP2518FD over SPI) and transceiver.
- Onboard IMU part (SPI, ≥1 kHz, low noise), required for VIO.
- Per-module resource check against [PLAN.md](../PLAN.md#per-module-resource-estimate), and whether a newer SoC (e.g. AMD Versal AI Edge Gen 2) is available in a small module yet.
- Module-size check: the K26 footprint against a Limelight-sized housing, and smaller EV-class SOMs as alternatives (EG parts have no video encoder).
- Recording check: VCU support in the current tools, and an M.2 or USB SSD on the dev kit.
- Purchase list: dev kit, a matched pair of mono global-shutter MIPI sensors (two pairs to test multiple modules), a color global-shutter pair, cables/adapters, a CAN-FD transceiver breakout, an M.2 2230 NVMe SSD.
- Repo scaffolding: `rtl/`, `sim/`, `platform/`, `rpu/`, `apu/`, `ml/`, `model/`, `hardware/`, with a README stub in each.
- GitHub Actions: Verible lint, Verilator + cocotb sims, `model/` and `apu/` unit tests. Bitstream builds on a nightly or self-hosted job.

## Exit criteria

- [ ] Module, dev kit and sensors chosen, with the report filled in and all numbers checked against datasheets.
- [ ] Confirmed that the current Vitis AI release supports a DPU on the chosen part (or the report names the object-detection fallback).
- [ ] CI runs on every PR and is green.
- [ ] A "blinky" bitstream loads from Linux on the dev kit.

## Risks

- Vitis AI's focus has been shifting to newer AMD chips; check its support for Zynq UltraScale+ before buying.
- Dev kits vary in how many MIPI camera ports they have; a stereo pair needs two. Check the kit before buying.
