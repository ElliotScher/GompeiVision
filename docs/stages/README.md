# Stages

Each stage has a goal, deliverables, exit criteria and risks. A stage is done when **all** of its exit criteria pass and the results (especially measured latency, resource and cost numbers) are written into its doc. See [PLAN.md](../PLAN.md) for the overall design and the [README](../../README.md) for the roadmap checklist.

| Stage | Title | Depends on |
|---|---|---|
| [0a](stage-0a-dev-environment.md) | Dev environment and toolchain checks | — |
| [0b](stage-0b-hardware-and-scaffolding.md) | Hardware confirmation, cost check, scaffolding, simulation CI | 0a |
| [1](stage-1-baseline-and-golden-models.md) | Latency baseline and golden models | 0a |
| [2](stage-2-platform-bring-up.md) | Platform bring-up: Linux, ntcore, real-time cores, shared memory | 0b |
| [3](stage-3-camera-capture.md) | Camera capture, timestamps, FrameBus, calibration, setup stream, recording | 2 |
| [4](stage-4-apriltag-detector.md) | Streaming AprilTag detector | 1, 3 |
| [5](stage-5-pose-and-mvp.md) | CAN-FD corners, robot-side PnP and **MVP parity** | 4 (CAN-FD and the robot library can start after 3, with NAPA) |
| [6](stage-6-multi-module.md) | Multiple modules: fusion and joint solves on the robot | 5 |
| [7](stage-7-object-detection.md) | Object detection (DPU) and localization | 3 |
| [8](stage-8-tagslam.md) | TagSLAM field calibration | 5 (better after 6) |
| [9](stage-9-module-hardware.md) | Module hardware: carrier and camera boards, machined enclosure, SSD, competition hardening | 5 (layout can start after 0b) |

Because the software track (Linux, ML, robot library) and the hardware track (FPGA logic) are separate, Stage 7 can start as soon as capture works, the robot library can be written against NAPA's corners before the FPGA detector exists, and Stage 8 runs on a desktop with recorded data.

Throughout: **[PL]** FPGA logic, **[RPU]** real-time Cortex-R5F cores, **[APU]** Cortex-A53 cores running Linux, **[Robot]** robot code on the SystemCore (see [Where each block runs](../PLAN.md#where-each-block-runs)).
