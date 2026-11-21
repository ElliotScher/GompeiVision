# GompeiVision
![logo](logo3.png)

**A self-contained, FPGA-accelerated vision coprocessor for FRC, built by Team 190, Gompei and the H.E.R.D.**

> **Status:** pre-alpha (planning). Nothing in this README is ready to use yet. The CPU pipeline in [`src/`](src/) (documented in [docs/legacy-cpu-pipeline.md](docs/legacy-cpu-pipeline.md)) is still our competition system. See [Roadmap and status](#roadmap-and-status).

## What GompeiVision is and why it exists

COTS vision units like Limelight, and our own CPU version of GompeiVision, follow the same pattern: a USB or MIPI camera fills a frame buffer, a general-purpose CPU wakes up, copies the whole frame, and only then starts looking for AprilTags. That means at least one full frame of transfer delay, plus processing time that changes from frame to frame with OS scheduling, cache misses and the number of tags in view.

GompeiVision runs on an **SoC FPGA**: one chip with FPGA logic, real-time cores and a Linux-capable ARM CPU sharing the same memory. The latency-critical work (camera capture, AprilTag detection, corner output) runs in **hardware and on real-time cores**. Pixels are processed as they come off the sensor, so a tag's corners are found a millisecond or two after the last row of the tag is read out, and go straight to the robot over **CAN-FD**. The robot controller (SystemCore) solves the pose, where it already has the gyro, the tag map and every camera's mounting position. The heavier, irregular work (object detection, networking, recording) runs on the module's **ARM CPU under Linux**, where we can use normal C++, WPILib's `ntcore` and existing libraries. The module also publishes the same NetworkTables topics we use today.

The system is built from **single-lens modules**, one product made as cheaply as we can. Each module is a self-contained, Limelight-sized unit (a few inches on each side) with one global-shutter camera, a Kria K26 SoC FPGA, built-in object detection and on-module video recording, in a housing we machine ourselves, for roughly $470–580. A robot carries as many modules as it needs, and the robot fuses all of their tag corners.

What we expect to gain, in order of importance:

1. **Deterministic, low-jitter latency.** The tag path has a fixed timing budget, so the time from exposure to measurement is nearly constant and can be compensated exactly on the robot.
2. **Exact hardware timestamps.** The FPGA triggers the sensor and records the start of exposure in hardware. No guessing from a driver callback.
3. **Lower absolute latency.** We estimate exposure + readout + ~1.5–3 ms to corners at the robot, compared with an estimated 20–40 ms for the USB/CPU path. Both figures get *measured* in Stage 1 before we believe them.
4. **Better pose solves on the robot,** using every module's corners and the live gyro in one place, plus object detection and recording on every module at a lower cost than COTS units with object detection.

## Is this feasible?

**Yes.** Each block runs on the part of the system that suits it, so nothing has to be forced into hardware:

| Feature | Runs on | Notes |
|---|---|---|
| Camera capture, timestamps, frame routing | FPGA logic | MIPI CSI-2 receive, hardware trigger and timestamping are what FPGAs do best |
| AprilTag (36h11) detection | FPGA logic | Streaming threshold → connected components → quad fit → decode, combining ideas from AprilTag 3, NAPA and ArUco. The main latency win and the hardest custom hardware. NAPA on the ARM CPU is the fallback and cross-check |
| Corner output | Real-time cores + CAN-FD | Undistorted tag corners sent to SystemCore within microseconds, with no Linux in the way |
| Pose estimation (IPPE, multi-tag, heading-constrained) | Robot code on SystemCore | Small numeric problems, tens of µs per tag, with the robot's gyro and every module's corners. The module also solves its own pose for NetworkTables |
| Object detection | Prebuilt accelerator in FPGA logic, run from Linux | AMD's neural-network accelerator (Vitis AI DPU), built into every module. No custom accelerator to build |
| Object localization | ARM CPU (Linux) | Ground-plane projection |
| NetworkTables, config, setup stream | ARM CPU (Linux) | Uses WPILib's real `ntcore` library |
| Video recording | Hardware video encoder + on-module eMMC | H.265, written to each module's own eMMC |
| TagSLAM | Desktop, from logs | Offline field calibration: refines the tag map from every module's logged corners |

The real risks are not technical limits. They are **people and time**: HDL skills are rare on student teams, the AprilTag hardware takes several months to a season, and debugging hardware is slower than debugging C++. The full analysis, latency budget and risk register are in [docs/PLAN.md](docs/PLAN.md).

## Design principles

- **Stream, don't buffer.** The tag path never waits for a full frame. Blocks pass pixels and detections over AXI-Stream as they're produced.
- **Hardware for the fast path, Linux for the rest.** The tag path is FPGA logic plus real-time cores, straight to CAN-FD. Linux never touches it.
- **Solve where the state is.** Pose solving needs the gyro, tag map and camera positions, which live on the robot, so PnP runs in robot code. The module sends corners, quality and timing.
- **One loop on the robot.** Everything a module sends is used in the loop it arrives in and then dropped; only the pose estimator keeps history.
- **Same contract as today.** The module also speaks the NetworkTables topics the CPU version uses (`/cameras/<hardware_id>/config`, `/field/tag_N`, `/cameras/<hardware_id>/output/observations`), so existing code and dashboards keep working.
- **Golden-model parity.** Every hardware block has a C++ reference model (starting from the code in [`src/`](src/)). Simulations must match it bit for bit, or within a documented tolerance for floating-point work.
- **Measure before claiming.** Latency and cost numbers in these docs are estimates until a stage measures them.
- **Independent pipelines.** Each module, and each block on it, can fail or be restarted without affecting the others. A crash in a Linux process never stops tag detection.

## Where each block runs

The SoC FPGA has three kinds of compute, all sharing the same DDR memory, and the robot controller adds a fourth:

1. **FPGA logic (PL).** Capture, timestamps, frame routing, AprilTag detection, the downscaler and the object-detection accelerator, and the CAN-FD controller.
2. **Real-time cores (RPU, Cortex-R5F, no Linux).** Corner undistortion, CAN-FD corner frames, and module-side PnP for NetworkTables.
3. **Application CPU (APU, Cortex-A53, Linux).** NetworkTables via `ntcore`, config and field layout, object-detection control and localization, storage, the setup-mode stream and video recording (through the chip's hardware encoder).
4. **Robot controller (SystemCore).** PnP over every module's corners, heading-constrained solves and the pose estimator, in our robot library.

Rules:

- **The tag path never waits on Linux.** Capture → detection → corners → CAN-FD runs in FPGA logic and on the real-time cores. Measurement *time* always comes from the hardware exposure timestamp, so delivery jitter never corrupts it.
- **The tag path boots without Linux,** so after a brownout it's back within seconds while Linux is still starting.
- **Every block exchanges messages in one versioned format** (the FrameBus messages), passed through shared memory. Blocks can move between tiers without changes upstream or downstream.
- **A block moves only after a stage measures it missing its budget**, and the stage doc records the measurement.

## Architecture

This maps the macro design in [`GompeiVisionStructureDiagram.dot`](GompeiVisionStructureDiagram.dot) onto the chip. `[PL]` is FPGA logic, `[RPU]` the real-time cores, `[APU]` the Linux CPU.

```
 One MIPI CSI-2 global-shutter sensor (mono or color)
        │  trigger + exposure strobe
        ▼
┌──────────────────────────── SoC FPGA (Kria K26, Zynq UltraScale+) ─────────────────────────────┐
│ [PL] Capture: CSI-2 RX · exposure timestamp · line buffers                                     │
│        │                                                                                       │
│ [PL] FrameBus: AXI-Stream router ──► shared DDR frame store                                    │
│        │                                                                                       │
│        ├─► [PL] AprilTag: threshold → CCL → quads → decode ─► [RPU] corners ──────────┐        │
│        │                         [RPU] module-side PnP (same detections) ───┐         │        │
│        └─► [PL] downscaler → DPU ◄─ [APU] object detection + localization ──┤         │        │
│                                                                             ▼         │        │
│ [APU] Linux: ntcore · config · field layout · eMMC recording (VCU) · setup stream     │        │
└─────────────────────────────────────────────────────────────────────────────┬─────────┬────────┘
                                                                              │         │
                                             GbE (NT: poses, objects, status) │         │ CAN-FD (tag corners)
                                                                              ▼         ▼
   SystemCore robot code: robot library (PnP over all modules, gyro at capture time)
                          → pose estimator · RobotState
```

This diagram is one module. A robot carries several, all on one dedicated CAN-FD bus and the robot's Ethernet. Robot code reads every module's corners each loop, solves the poses and fuses them.

## Repository layout (planned)

- [`src/`](src/): the current CPU pipeline (C++, OpenCV, AprilTag, ntcore). Still the competition system, the starting point for the golden models, and the first thing we run on the new board's ARM cores.
- `model/`: C++ golden reference models for each hardware block, plus a frame-replay harness.
- `rtl/`: SystemVerilog for the FPGA logic (capture, FrameBus, AprilTag, downscaler, CAN-FD).
- `sim/`: Verilator + cocotb testbenches that compare RTL with `model/` on recorded frames.
- `platform/`: Vivado block design, device tree, and the Linux image build.
- `rpu/`: real-time core firmware (FreeRTOS or bare metal, OpenAMP/RPMsg): corner output, CAN-FD, module-side PnP.
- `apu/`: Linux C++ apps: NT publisher, config, object detection, recording, storage, setup stream.
- `robot/`: the robot library (Java, optional C++ core over JNI): CAN-FD reader, PnP, heading-constrained and multi-module solves.
- `ml/`: object-detection datasets, training and quantization scripts.
- `tools/tagslam/`: the offline TagSLAM field-calibration tool.
- `hardware/`: the custom carrier board with the camera sensor on it (KiCad) and the machined enclosure (FreeCAD).
- `scripts/`: `bootstrap.sh` and `doctor.sh` for toolchain setup and checks.
- `tools/versions.env`: pinned toolchain versions.
- [`docs/`](docs/): [the plan](docs/PLAN.md), [stage specs](docs/stages/README.md) and [the legacy CPU docs](docs/legacy-cpu-pipeline.md).

## Hardware summary (target)

The exact parts get confirmed in Stage 0b. The criteria, candidates and cost estimate are in [docs/PLAN.md](docs/PLAN.md#hardware).

- **SoC FPGA:** an AMD **Kria K26** system-on-module (Zynq UltraScale+ EV: quad Cortex-A53, dual Cortex-R5F, FPGA logic, hardware video encoder, shared DDR4). We prototype on the off-the-shelf dev kit, then design our own small carrier board for the same module.
- **Camera:** one global-shutter MIPI CSI-2 sensor per module (MIPI rather than USB, so rows stream straight into the FPGA logic as they're read out): OV9281 mono or OV9782 color at 1280×800 and 120 FPS, with AR0234 (1920×1200) as an upgrade. Our own small camera boards and tested M12 lenses.
- **Object detection:** built into every module, on the FPGA's DPU accelerator.
- **Networking:** CAN-FD to SystemCore for tag corners; Gigabit Ethernet for NetworkTables, config, video and updates.
- **Storage:** eMMC for the Linux image, config, network weights and the tag map (no SD card on the robot).
- **Recording:** match video and logs on a data partition of the SOM's 16 GB eMMC; raw datasets stream over Ethernet (no SSD).
- **Size:** about a Limelight, roughly 3.5 × 2.8 × 1.5 in, in an aluminum housing machined on our own mill that is also the heatsink.
- **Power:** robot 12 V through a discrete buck-boost regulator that holds through brownouts, with a fuse/breaker on the PDH, following the current FRC game manual.
- **Cost:** roughly $470–580 per module (estimate; target $400–580).

## Development setup

On Ubuntu 24.04+ or Debian 13 (WSL2 works on Windows), install the tools for the tracks you work on, then check them:

```sh
scripts/bootstrap.sh --hardware --robot   # or --rpu, --apu, --ml; no flags = every track
scripts/doctor.sh --selftest              # checks versions and runs a Verilator + cocotb test
```

The tracks are:
- **Hardware:** Verilator, Verible, cocotb.
- **RPU:** ARM bare-metal GCC.
- **APU:** AArch64 Linux cross-compiler, CMake, Ninja.
- **Robot:** JDK for WPILib 2027.
- **ML:** Docker and the Vitis AI image.

Versions are pinned in [`tools/versions.env`](tools/versions.env). The AMD tools (Vivado, Vitis, PetaLinux) can't be redistributed and are optional on student machines. See [docs/dev-setup.md](docs/dev-setup.md) for details, manual setup and troubleshooting.

The legacy CPU pipeline still builds as before, inside the X86 or ARM Docker containers ([`Dockerfile.X86`](Dockerfile.X86), [`Dockerfile.ARM`](Dockerfile.ARM)), and packages as a `.deb`.

## Roadmap and status

The full plan is in [docs/PLAN.md](docs/PLAN.md), with per-stage detail in [docs/stages/](docs/stages/README.md).

- [x] **0a**: Dev environment and toolchain checks
- [ ] **0b**: Hardware confirmation, cost check, repo scaffolding and simulation CI
- [ ] **1**: Latency baseline of the CPU system and golden models
- [ ] **2**: Platform bring-up: Linux, ntcore, real-time cores, shared memory
- [ ] **3**: Camera capture, hardware timestamps, FrameBus, calibration, setup-mode stream and video recording
- [ ] **4**: Streaming AprilTag detector in FPGA logic
- [ ] **5**: CAN-FD corners, robot-side PnP and **MVP: output parity with the CPU pipeline**
- [ ] **6**: Multiple modules: fusion and joint solves on the robot
- [ ] **7**: Object detection (DPU) and object localization
- [ ] **8**: TagSLAM field calibration
- [ ] **9**: Module hardware: carrier board with the camera on it, machined enclosure and competition hardening

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
