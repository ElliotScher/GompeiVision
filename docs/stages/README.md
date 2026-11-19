# Stages

Each stage has a goal, deliverables, exit criteria and risks. A stage is done when **all** of its exit criteria pass and the results (especially measured latency and resource numbers) are written into its doc. See [PLAN.md](../PLAN.md) for the overall design and the [README](../../README.md) for the roadmap checklist.

| Stage | Title | Depends on |
|---|---|---|
| [0a](stage-0a-dev-environment.md) | Dev environment and toolchain checks | — |
| [0b](stage-0b-hardware-and-scaffolding.md) | Hardware confirmation, scaffolding, simulation CI | 0a |
| [1](stage-1-baseline-and-golden-models.md) | Latency baseline and golden models | 0a |
| [2](stage-2-platform-bring-up.md) | Platform bring-up: Linux, ntcore, real-time cores, shared memory | 0b |
| [3](stage-3-camera-capture.md) | Camera capture, timestamps, FrameBus, setup stream, recording | 2 |
| [4](stage-4-apriltag-detector.md) | Streaming AprilTag detector | 1, 3 |
| [5](stage-5-pose-and-mvp.md) | Pose estimation and **MVP parity** | 4 |
| [6](stage-6-multi-camera-stereo.md) | Stereo module and multi-module fusion | 5 |
| [7](stage-7-object-detection.md) | Object detection (DPU) and localization | 3 |
| [8](stage-8-tagslam.md) | TagSLAM and refined tag map | 5 (6 for stereo) |
| [9](stage-9-mcl.md) | MCL localization | 8 |
| [10](stage-10-vslam.md) | VSLAM | 6 |
| [11](stage-11-custom-hardware.md) | Module hardware: carrier board, enclosure, SSD, competition hardening | 5 (layout can start after 6) |

Because the software track (Linux, ML) and the hardware track (FPGA logic) are separate, Stage 7 can start as soon as capture works, and Stages 8 and 10 can be prototyped on a desktop with recorded data before the hardware is ready.

Throughout: **[PL]** FPGA logic, **[RPU]** real-time Cortex-R5F cores, **[APU]** Cortex-A53 cores running Linux (see [Where each block runs](../PLAN.md#where-each-block-runs)).
