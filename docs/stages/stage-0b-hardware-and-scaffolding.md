# Stage 0b: Hardware confirmation, scaffolding, simulation CI

## Goal

Confirm the SoC FPGA module, dev kit and sensors with real numbers, and set up the repo layout and CI so every later stage has a place to put code and tests.

## Deliverables

- **Hardware report** in this doc: the Kria K26 dev kits (KV260, KR260) and any other candidates against the [selection criteria](../PLAN.md#selection-criteria-stage-0b): MIPI camera ports, logic/DSP/on-chip RAM, DPU support in the current Vitis AI release, free-tier toolchain coverage, price, availability.
- A first resource estimate per PL block (AprilTag pipeline, MCL scoring, DPU, capture per camera) against the chosen part, with ≥30% headroom.
- Purchase list: dev kit, 2+ mono global-shutter MIPI sensors, 1 color global-shutter sensor, cables/adapters.
- Repo scaffolding: `rtl/`, `sim/`, `platform/`, `rpu/`, `apu/`, `ml/`, `model/`, `hardware/`, with a README stub in each.
- GitHub Actions: Verible lint, Verilator + cocotb sims, `model/` and `apu/` unit tests. Bitstream builds on a nightly or self-hosted job.

## Exit criteria

- [ ] Module, dev kit and sensors chosen, with the report filled in and all numbers checked against datasheets.
- [ ] Confirmed that the current Vitis AI release supports a DPU on the chosen part (or the report names the object-detection fallback).
- [ ] CI runs on every PR and is green.
- [ ] A "blinky" bitstream loads from Linux on the dev kit.

## Risks

- Vitis AI's focus has been shifting to newer AMD chips; check its support for Zynq UltraScale+ before buying.
- Dev kits vary in how many MIPI camera ports they have; stereo plus a color camera may need an adapter or a different kit.
