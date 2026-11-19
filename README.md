# GompeiVision
![logo](logo3.png)

**A self-contained, FPGA-accelerated vision coprocessor for FRC, built by Team 190, Gompei and the H.E.R.D.**

> **Status:** pre-alpha (planning). Nothing in this README is ready to use yet. The CPU pipeline in [`src/`](src/) (documented in [docs/legacy-cpu-pipeline.md](docs/legacy-cpu-pipeline.md)) is still our competition system. See [Roadmap and status](#roadmap-and-status).

## What GompeiVision is and why it exists

COTS vision units like Limelight, and our own CPU version of GompeiVision, follow the same pattern: a USB or MIPI camera fills a frame buffer, a general-purpose CPU wakes up, copies the whole frame, and only then starts looking for AprilTags. That means at least one full frame of transfer delay, plus processing time that changes from frame to frame with OS scheduling, cache misses and the number of tags in view.

GompeiVision runs on an **SoC FPGA**: one chip with FPGA logic, real-time cores and a Linux-capable ARM CPU sharing the same memory. The latency-critical work (camera capture, AprilTag detection, pose solving, MCL) runs in **hardware and on real-time cores**. Pixels are processed as they come off the sensor, so a tag's corners are found a millisecond or two after the last row of the tag is read out. The heavier, irregular work (object detection, TagSLAM, VSLAM, networking) runs on the **ARM CPU under Linux**, where we can use normal C++, WPILib's `ntcore` and existing libraries. The unit talks to the robot over the same NetworkTables contract we use today.

The system is built from **stereo modules**. Each module is a self-contained, Limelight-sized unit (a few inches on each side) with exactly two global-shutter lenses, its own SoC FPGA and its own SSD. A robot carries as many modules as it needs. Each one detects tags, runs stereo VSLAM and TagSLAM, records its own video, and reports field-frame poses that the robot's pose estimator fuses.

What we expect to gain, in order of importance:

1. **Deterministic, low-jitter latency.** The tag path has a fixed timing budget, so the time from exposure to measurement is nearly constant and can be compensated exactly on the robot.
2. **Exact hardware timestamps.** The FPGA triggers the sensor and records the start of exposure in hardware. No guessing from a driver callback.
3. **Lower absolute latency.** We estimate exposure + readout + ~2–3 ms, compared with an estimated 20–40 ms for the USB/CPU path. Both figures get *measured* in Stage 1 before we believe them.
4. **Higher FPS and onboard stereo**, with a hardware-synchronized lens pair in every module, and more cameras by adding modules.

## Is this feasible?

**Yes.** Each block runs on the part of the chip that suits it, so nothing has to be forced into hardware:

| Feature | Runs on | Notes |
|---|---|---|
| Camera capture, timestamps, frame routing | FPGA logic | MIPI CSI-2 receive, hardware trigger and timestamping are what FPGAs do best |
| AprilTag (36h11) detection | FPGA logic | Streaming threshold → connected components → quad fit → decode. The main latency win and the hardest custom hardware |
| Pose estimation (IPPE, SQPnP, tag angles, stereo) | Real-time cores | Tiny numeric problems; well under 1 ms, with no Linux scheduling in the way |
| MCL | FPGA logic + real-time cores | Scoring thousands of particles in parallel suits hardware |
| Object detection | Prebuilt accelerator in FPGA logic, run from Linux | AMD's neural-network accelerator (Vitis AI DPU), used like a library. No custom accelerator to build |
| TagSLAM, VSLAM, object localization | ARM CPU (Linux) | Iterative, memory-heavy work. VSLAM can build on existing open-source SLAM code |
| NetworkTables, config, maps, setup stream | ARM CPU (Linux) | Uses WPILib's real `ntcore` library |
| Video recording | Hardware video encoder + M.2 SSD | Both lenses, H.265, written to each module's own SSD |

The real risks are not technical limits. They are **people and time**: HDL skills are rare on student teams, the AprilTag hardware takes several months to a season, and debugging hardware is slower than debugging C++. The full analysis, latency budget and risk register are in [docs/PLAN.md](docs/PLAN.md).

## Design principles

- **Stream, don't buffer.** The tag path never waits for a full frame. Blocks pass pixels and detections over AXI-Stream as they're produced.
- **Hardware for the fast path, Linux for the rest.** The tag path is FPGA logic plus real-time cores. Linux only handles the final publish step on that path, in a real-time thread.
- **Same contract as today.** The unit speaks the NetworkTables topics the CPU version uses (`/cameras/<hardware_id>/config`, `/field/tag_N`, `/cameras/<hardware_id>/output/observations`), so robot code doesn't change.
- **Golden-model parity.** Every hardware block has a C++ reference model (starting from the code in [`src/`](src/)). Simulations must match it bit for bit, or within a documented tolerance for floating-point work.
- **Measure before claiming.** Latency numbers in these docs are estimates until a stage's test rig measures them.
- **Independent pipelines.** Each camera pipeline and estimator can fail or be restarted without affecting the others. A crash in a Linux process never stops tag detection.

## Where each block runs

The SoC FPGA has three kinds of compute, all sharing the same DDR memory:

1. **FPGA logic (PL).** Capture, timestamps, frame routing, AprilTag detection, MCL particle scoring, optional FAST/ORB features for VSLAM, and the object-detection accelerator.
2. **Real-time cores (RPU, Cortex-R5F, no Linux).** Pose solving (PnP, tag angles, stereo PnP), MCL resampling, and passing finished tag measurements to the publisher.
3. **Application CPU (APU, Cortex-A53, Linux).** NetworkTables via `ntcore`, config and field layout, object-detection control and localization, TagSLAM, VSLAM back end, map storage, the setup-mode stream and video recording (through the chip's hardware encoder).

Rules:

- **The tag path never waits on ordinary Linux work.** It only reaches Linux for the final publish, in a high-priority real-time thread. Measurement *time* always comes from the hardware exposure timestamp, so delivery jitter never corrupts it.
- **If publish jitter is measured too high, the tag path gets its own path that skips Linux,** driven by the real-time cores: raw UDP through a small Ethernet MAC in the FPGA logic, or CAN-FD on a dedicated SystemCore bus.
- **The tag path boots without Linux,** so after a brownout it's back within seconds while Linux is still starting.
- **Every block exchanges messages in one versioned format** (the FrameBus messages), passed through shared memory. Blocks can move between tiers without changes upstream or downstream.
- **A block moves only after a stage measures it missing its budget**, and the stage doc records the measurement.

## Architecture

This maps the macro design in [`GompeiVisionStructureDiagram.dot`](GompeiVisionStructureDiagram.dot) onto the chip. `[PL]` is FPGA logic, `[RPU]` the real-time cores, `[APU]` the Linux CPU.

```
 Stereo pair: 2 MIPI CSI-2 global-shutter sensors on one hardware trigger
        │  trigger + exposure strobe
        ▼
┌───────────────────────── SoC FPGA (e.g. Zynq UltraScale+) ─────────────────────────┐
│ [PL] Capture: CSI-2 RX · exposure timestamp · line buffers                         │
│        │                                                                           │
│ [PL] FrameBus: AXI-Stream router ──► shared DDR frame store                        │
│        │                                                                           │
│        ├─► [PL] AprilTag: threshold → CCL → quads → decode ─► [RPU] PnP, angles,   │
│        │                                                       stereo PnP ──┐      │
│        ├─► [PL] MCL particle scoring ─► [RPU] resampling ─► MCL pose ───────┤      │
│        ├─► [PL] DPU accelerator ◄─ [APU] object detection + localization ───┤      │
│        └─► [PL] FAST/ORB (optional) ─► [APU] VSLAM back end, geometric map ─┤      │
│                                        [APU] TagSLAM ─► refined tag map ────┤      │
│                                                                             ▼      │
│ [APU] Linux: ntcore publisher (real-time thread) · config · field layout          │
│              maps (eMMC) · M.2 recording (VCU) · setup stream                      │
└──────────────────────────────────────┬─────────────────────────────────────────────┘
                                       │ GbE
                                       ▼
   Robot radio ─► roboRIO / SystemCore: RobotState
                  ◄─ RobotData (heading, module positions, tag map)
```

This diagram is one module. A robot carries several, each connected to the robot by Ethernet and CAN-FD, and the robot's pose estimator fuses their field-frame poses.

## Repository layout (planned)

- [`src/`](src/): the current CPU pipeline (C++, OpenCV, AprilTag, ntcore). Still the competition system, the starting point for the golden models, and the first thing we run on the new board's ARM cores.
- `model/`: C++ golden reference models for each hardware block, plus a frame-replay harness.
- `rtl/`: SystemVerilog for the FPGA logic (capture, FrameBus, AprilTag, MCL scoring, features).
- `sim/`: Verilator + cocotb testbenches that compare RTL with `model/` on recorded frames.
- `platform/`: Vivado block design, device tree, and the Linux image build.
- `rpu/`: real-time core firmware (FreeRTOS or bare metal, OpenAMP/RPMsg): PnP, MCL resampling.
- `apu/`: Linux C++ apps: NT publisher, config, object detection, TagSLAM, VSLAM, storage, setup stream.
- `ml/`: object-detection datasets, training and quantization scripts.
- `hardware/`: custom carrier board (KiCad) and enclosure (FreeCAD).
- `scripts/`: `bootstrap.sh` and `doctor.sh` for toolchain setup and checks.
- `tools/versions.env`: pinned toolchain versions.
- [`docs/`](docs/): [the plan](docs/PLAN.md), [stage specs](docs/stages/README.md) and [the legacy CPU docs](docs/legacy-cpu-pipeline.md).

## Hardware summary (target)

The exact parts get confirmed in Stage 0b. The criteria and candidates are in [docs/PLAN.md](docs/PLAN.md#hardware).

- **SoC FPGA:** AMD Zynq UltraScale+ MPSoC (quad Cortex-A53, dual Cortex-R5F, FPGA logic, shared DDR4). We prototype on an off-the-shelf **Kria K26** system-on-module and dev kit, and later design our own small carrier board for the same module, one per stereo module.
- **Cameras:** global-shutter MIPI CSI-2 sensors (MIPI rather than USB, so rows stream straight into the FPGA logic as they're read out). Each module has exactly two: a monochrome (OV9281-class) stereo pair on one hardware trigger, at the far ends of the module for the widest baseline. Object detection uses a module variant whose pair is color global shutter (AR0234-class).
- **Networking:** Gigabit Ethernet to the robot radio; optional CAN-FD fast path to SystemCore.
- **Storage:** eMMC for the Linux image, config, network weights and saved maps (no SD card on the robot).
- **Recording:** an M.2 2230 NVMe SSD in every module for video and logs.
- **Size:** about a Limelight, roughly 3.5–4.5 × 2.5–3 × 1.5–2 in, with the aluminum housing as the heatsink and a small fan.
- **Power:** robot 12 V through a buck-boost regulator that holds through brownouts, with a fuse/breaker on the PDH, following the current FRC game manual.

## Development setup

Not available yet. Stage 0a will add `scripts/bootstrap.sh` and `scripts/doctor.sh` to install and check:

- AMD Vivado and Vitis (free tier), and the Linux image tooling (PetaLinux/Yocto, or the vendor Ubuntu image for Kria)
- Vitis AI (in its Docker container) for the object-detection model
- Verilator, cocotb and Python 3 for simulation
- AArch64 Linux and ARM bare-metal cross-compilers
- CMake and a host C++ compiler for `model/` and the legacy pipeline

The legacy CPU pipeline still builds as before, inside the X86 or ARM Docker containers ([`Dockerfile.X86`](Dockerfile.X86), [`Dockerfile.ARM`](Dockerfile.ARM)), and packages as a `.deb`.

## Roadmap and status

The full plan is in [docs/PLAN.md](docs/PLAN.md), with per-stage detail in [docs/stages/](docs/stages/README.md).

- [ ] **0a**: Dev environment and toolchain checks
- [ ] **0b**: Hardware confirmation, repo scaffolding and simulation CI
- [ ] **1**: Latency baseline of the CPU system and golden models
- [ ] **2**: Platform bring-up: Linux, ntcore, real-time cores, shared memory
- [ ] **3**: Camera capture, hardware timestamps, FrameBus, setup-mode stream and video recording
- [ ] **4**: Streaming AprilTag detector in FPGA logic
- [ ] **5**: Pose estimation and **MVP: output parity with the CPU pipeline**
- [ ] **6**: Stereo module (synced lens pair, stereo PnP) and multi-module fusion
- [ ] **7**: Object detection (DPU) and object localization
- [ ] **8**: TagSLAM and refined tag map storage
- [ ] **9**: MCL localization
- [ ] **10**: VSLAM
- [ ] **11**: Module hardware: carrier board, enclosure, SSD and competition hardening

## Inspiration

GompeiVision was heavily inspired by FRC Team 6328, Mechanical Advantage, and their vision system **[Northstar](https://github.com/Mechanical-Advantage/RobotCode2025Public/tree/main/northstar)**, along with their [computer vision conference talk](https://www.youtube.com/watch?v=fwEUqL_waPk&list=PLkZ6_Ld1x9Y-vRuH1q1Nt6vXIk8j2np7P&index=5). We are grateful for the ideas they have shared with the FRC community.

## A note to other FRC teams

We are proud to share this work, but please keep in mind:

- **Highly customized:** GompeiVision is built for our robots, sensors and software framework. It is not a general-purpose solution.
- **Unsupported:** during build and competition season our focus is our own robot. We **can't guarantee support, answer questions or accept feature requests**.

You're welcome to fork this repository and adapt it, but you'll be responsible for implementing, debugging and maintaining your version.

## Recommended alternatives

For a robust, well-supported, community-tested vision solution, we recommend:

- **[Limelight](https://limelightvision.io/)**: a popular plug-and-play vision camera for FRC.
- **[PhotonVision](https://photonvision.org/)**: powerful open-source vision software that runs on many coprocessors.

## License

[GPL-3.0](LICENSE).
