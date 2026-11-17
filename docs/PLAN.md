# GompeiVision FPGA plan

This document covers whether an FPGA-accelerated GompeiVision is feasible, how each block would be built, and how we'll know it works. The [README](../README.md) has the summary; per-stage specs are in [stages/](stages/README.md).

Every latency, resource and timing figure here is an **estimate**. Figures become facts only when a stage measures them, and the stage doc records the result.

## Contents

1. [Goals and non-goals](#goals-and-non-goals)
2. [Feasibility by block](#feasibility-by-block)
3. [Where each block runs](#where-each-block-runs)
4. [Latency budget](#latency-budget)
5. [Bandwidth and memory budget](#bandwidth-and-memory-budget)
6. [Hardware](#hardware)
7. [Toolchain](#toolchain)
8. [Block design](#block-design)
9. [Interfaces and message formats](#interfaces-and-message-formats)
10. [Robot interface (NetworkTables contract)](#robot-interface-networktables-contract)
11. [Verification strategy](#verification-strategy)
12. [Risk register](#risk-register)
13. [FRC rules checklist](#frc-rules-checklist)
14. [Team and skills](#team-and-skills)
15. [Notes on the structure diagram](#notes-on-the-structure-diagram)

## Goals and non-goals

**Goals**

- One self-contained unit (board + cameras + enclosure) that implements the macro architecture in [`GompeiVisionStructureDiagram.dot`](../GompeiVisionStructureDiagram.dot): capture, FrameBus, Mono/Stereo TagPnP, object recognition and localization, TagSLAM, MCL, VSLAM, NetworkTables output and persistent map storage.
- Exposure-to-measurement latency on the tag path that is low **and** nearly constant, with hardware exposure timestamps.
- Drop-in replacement for the CPU pipeline: robot code keeps using the same NetworkTables topics.
- At least 2 cameras per unit at 1280×800, ≥100 FPS for AprilTags (to be confirmed in Stage 0b against the chosen part).

**Non-goals**

- A general-purpose product for other teams. See the README's note to other teams.
- Putting everything in FPGA logic. Blocks go where they fit best (see [Where each block runs](#where-each-block-runs)).
- Writing our own NetworkTables client or neural-network accelerator when maintained ones exist.
- Replacing the CPU pipeline before the FPGA build reaches output parity (Stage 5).

## Feasibility by block

Legend: **[PL]** FPGA logic, **[RPU]** real-time Cortex-R5F cores, **[APU]** Cortex-A53 cores running Linux.

| Diagram node | Implementation | Difficulty | Verdict and main risk |
|---|---|---|---|
| MonoCameras, StereoCameras | [PL] MIPI CSI-2 RX (vendor IP), hardware trigger and exposure-strobe timestamp counter; sensor I2C config from [APU] | Easy–moderate | Feasible. Risk: MIPI bring-up and signal integrity |
| MonoFrameBus, StereoFrameBus | [PL] AXI-Stream switch with per-consumer FIFOs and a DDR writer | Easy | Feasible |
| MonoTagPnP (detection) | [PL] streaming threshold → connected components → boundary clustering → quad fit → decode → edge refinement | Moderate–hard | Feasible. The core latency win, and the biggest piece of custom hardware. See [AprilTag detector](#apriltag-detector) |
| MonoTagPnP, StereoTagPnP (pose) | [RPU] IPPE-square and SQPnP, ported from the OpenCV calls in [`CameraPoseEstimator.cpp`](../src/cpp/estimator/CameraPoseEstimator.cpp) and [`SingleTagPoseEstimator.cpp`](../src/cpp/estimator/SingleTagPoseEstimator.cpp) | Easy–moderate | Feasible; expected well under 1 ms on an R5F with its double-precision FPU |
| TagAngleCalculator | [RPU] undistort corners, convert to angles | Easy | Feasible |
| MCL | [PL] parallel particle scoring + [RPU] resampling, motion model from RobotData | Moderate | Feasible; a natural fit for parallel hardware |
| ObjectRecognition | [PL] AMD's prebuilt DPU accelerator, run from [APU] via Vitis AI | Moderate | Feasible. Risk: Vitis AI support and DPU size on the chosen part; model quantization accuracy |
| ObjectLocalization | [APU] ground-plane projection with camera extrinsics | Easy | Feasible |
| TagSLAM | [APU] fixed-lag sparse factor graph (tag landmarks, robot keyframes, odometry from RobotData) using an existing solver library | Moderate | Feasible; small problem (one field's tags) |
| VSLAM, LocalGeometricMap | [APU] tracking, bundle adjustment and map management, built on an existing open-source SLAM system; optional [PL] FAST/ORB features | Hard | Feasible as a CPU workload, limited by A53 speed. Research stretch |
| PoseMeasurements, ObjectData, MapData | [APU] WPILib `ntcore` in a real-time thread; optional [PL]+[RPU] raw-UDP fast path | Easy–moderate | Feasible. Risk: Linux publish jitter (measured in Stage 5) |
| MapStorage, SavedVSLAMMap | [APU] files on eMMC/microSD | Easy | Feasible |
| Setup-mode stream (from CPU version) | [APU] MJPEG from the frame store (or the chip's hardware video encoder, if present) | Easy | Feasible |

**Overall:** feasible. The only large custom hardware block is the AprilTag detector (plus the smaller MCL scoring engine). Everything else is ported or existing software, or prebuilt IP.

## Where each block runs

The SoC FPGA has three kinds of compute sharing one DDR memory:

1. **FPGA logic (PL)** for anything that touches pixels at line rate or is massively parallel: capture, frame routing, AprilTag detection, MCL particle scoring, optional VSLAM features, and the DPU.
2. **Real-time cores (RPU)** for small, latency-critical numeric work that must not be delayed by Linux: PnP, tag angles, stereo PnP, MCL resampling.
3. **Linux CPU (APU)** for large, irregular, memory-hungry work and anything that benefits from existing libraries: `ntcore`, config, object-detection control, TagSLAM, VSLAM, storage, setup stream.

Rules:

- **The tag path never waits on ordinary Linux work.** Capture → detection → PnP runs entirely in PL and RPU. The finished measurement goes through shared memory to a high-priority real-time (PREEMPT_RT) publisher thread on the APU, which sends it with `ntcore`.
- **Measurement time comes from the hardware exposure timestamp**, converted to NT server time. Jitter in delivery never changes the timestamp the robot sees.
- **If Stage 5 measures publish jitter as too high**, add a small Ethernet MAC in PL driven by the RPU, sending a raw-UDP fast path that doesn't touch Linux at all.
- All blocks exchange the versioned [FrameBus messages](#interfaces-and-message-formats) through shared memory, so a block can move between tiers without changes upstream or downstream.
- A block moves tiers only after a stage measures it missing its budget. The stage doc records the measurement and the decision.

## Latency budget

### Today (CPU pipeline, estimated, measured in Stage 1)

| Step | Estimate |
|---|---|
| Exposure | 1–3 ms |
| Sensor readout + USB3 transfer of the full frame | ~1 frame period (8–17 ms at 120–60 FPS) |
| V4L2 dequeue, copy, grayscale conversion | 1–3 ms |
| AprilTag detection at 1280×720 on CPU | 5–15 ms, varies with scene |
| PnP + packing | <1 ms |
| NT publish and flush | 0–10 ms depending on batching |
| **Total exposure start → packet on the wire** | **~20–40 ms, with several ms of jitter** |

### SoC FPGA target (tag path)

| Step | Estimate |
|---|---|
| Exposure | 1–3 ms (same sensor physics) |
| Sensor readout | overlaps with processing; only the rows *after* the last tag's bottom edge are waiting time |
| Detection tail after the last tag row (component closes, quad fit, decode, refine) | 0.5–2 ms |
| PnP on RPU | <1 ms |
| Hand-off to APU real-time thread + `ntcore` flush | <0.5 ms typical, jitter to be measured |
| **Total exposure start → packet on the wire** | **~3–7 ms + remaining readout, with low jitter** |

Object detection, TagSLAM and VSLAM aren't on this path. They have their own, looser budgets in their stage docs.

### What this buys on the robot

The robot loop runs every 20 ms (faster with high-frequency odometry threads), and pose estimators already compensate for latency using the measurement timestamp. So:

- The **biggest wins are jitter and timestamp accuracy.** A measurement whose age is known to ±0.1 ms fuses better than one known to ±5 ms, especially when the robot is turning fast.
- Lower absolute latency helps auto-align settle faster and reduces how far back the estimator has to replay odometry.
- After the packet leaves the unit, the robot-side NT server and the robot loop phase add up to one loop period. The raw-UDP fast path only helps if robot code reads it from a separate thread or notifier.

Stage 1 builds an **LED glass-to-robot rig** (an LED the robot toggles, seen by the camera, timestamped on both sides) so both systems are measured the same way.

## Bandwidth and memory budget

| Stream | Rate |
|---|---|
| One mono camera, 1280×800, 8-bit, 120 FPS | ≈123 MB/s |
| Four mono cameras | ≈490 MB/s |
| Color camera for object detection, downscaled to e.g. 320×320 RGB at 30 FPS | ≈9 MB/s |
| DDR frame store writes + DPU weights/activations + Linux + VSLAM | a few GB/s at most |

Zynq UltraScale+ modules use a wide DDR4 interface shared by PL, RPU and APU. Bandwidth is not expected to be the limit, but the DPU and VSLAM compete with the frame store, so Stage 7 and Stage 10 measure memory contention and check that the tag path is unaffected. On-chip RAM (block RAM / UltraRAM) for line buffers, tile statistics, label tables and the DPU is the tighter budget; Stage 0b estimates it per block.

## Hardware

### Selection criteria (Stage 0b)

1. SoC FPGA with Linux-capable application cores, real-time cores and FPGA logic sharing DDR.
2. MIPI CSI-2 receive for at least 2 (ideally 4) cameras.
3. Enough logic, DSP and on-chip RAM for the AprilTag pipeline, MCL scoring and a DPU, with ≥30% headroom.
4. Vitis AI (or equivalent) supports a DPU on the part.
5. An off-the-shelf module and dev kit available now, so we can prototype before designing our own board.
6. Covered by the free tier of the vendor toolchain.
7. Cost within FRC per-item limits.

### Candidates

| Option | Why | Concerns |
|---|---|---|
| **AMD Kria K26 SOM + dev kit (KV260 or KR260)** (primary) | Zynq UltraScale+ MPSoC with A53 + R5F + PL + DDR4 on one module; dev kits with camera connectors and Ethernet; vendor Ubuntu and PetaLinux images; Vitis AI DPU examples | Confirm current Vitis AI support level, free-tier coverage, MIPI port count on the kit and price |
| Other Zynq UltraScale+ boards | Same chip family, same toolchain; possibly more camera ports | Usually more expensive, less community support |
| Separate CPU with NPU + small FPGA (e.g. an RK3588 board like the Orange Pi 5, plus an FPGA front-end) | Faster CPU cores and a built-in NPU for object detection | Two chips, a CPU↔FPGA link (PCIe/MIPI) to design, no shared memory; more board work |

Zynq UltraScale+ is the primary choice because shared memory and one toolchain make the CPU/FPGA split simple. The separate-chip option stays documented as a fallback if VSLAM or object detection turns out to need much faster CPU cores than the A53s.

All part-specific numbers (logic, DSP, RAM, MIPI ports, price) get filled in and verified in Stage 0b.

### Sensors

- **AprilTags:** monochrome global shutter, 1–2 MP, ≥100 FPS, MIPI CSI-2, external trigger and strobe output (OV9281-class).
- **Objects:** color global shutter (AR0234-class), run at lower FPS and downscaled for the DPU.
- **Stereo:** two identical sensors on one trigger line, rigid mount, factory-calibrated baseline.

## Toolchain

| Purpose | Tool |
|---|---|
| RTL | SystemVerilog |
| Simulation | Verilator, cocotb (Python testbenches) |
| FPGA build and block design | AMD Vivado (free tier) |
| RPU firmware | Vitis / ARM bare-metal GCC, FreeRTOS or bare metal, OpenAMP/RPMsg |
| Linux image | PetaLinux/Yocto, or the vendor Ubuntu image for Kria, with PREEMPT_RT |
| APU apps | C++20, CMake, WPILib `ntcore`/`wpimath` (same as [`src/`](../src/)), OpenCV, a factor-graph library for TagSLAM |
| Object detection | Vitis AI (training and quantization in its Docker container) |
| Golden models | C++ host build sharing code with [`src/`](../src/) and the AprilTag C library |
| CI | GitHub Actions: lint (Verible), Verilator + cocotb sims, golden-model and APU unit tests. Bitstream builds run nightly or on a self-hosted runner, since Vivado is large |
| Board and enclosure | KiCad, FreeCAD |

## Block design

### Capture

- MIPI CSI-2 RX from vendor IP, unpacked to 8-bit pixels on an AXI-Stream with start-of-frame and end-of-line markers.
- The PL drives each sensor's trigger input. A 64-bit free-running counter latches the exposure strobe; this is the measurement timestamp.
- Sensor setup (exposure, gain, resolution) over I2C from the APU, driven by `/cameras/<hardware_id>/config`.
- Each sensor has a fixed physical port, which fixes the "no serial number" problem we had with USB ThriftyCams: `hardware_id` comes from the port plus a board ID.

### FrameBus

- An AXI-Stream switch that fans each camera stream out to the consumers subscribed to it (tag pipeline, MCL, downscaler for the DPU, feature extractor, DDR writer).
- Every frame is also written to a DDR ring buffer that PL, RPU and APU can all read. Consumers never wait for this copy; it's for decode and refinement read-back (below), object detection, setup-mode streaming and VSLAM.
- Consumers that fall behind drop frames and count drops; they never back-pressure the sensor.

### AprilTag detector

Follows the AprilTag 3 algorithm used today ([`FiducialDetector.cpp`](../src/cpp/detector/FiducialDetector.cpp)) so results can be compared one to one.

1. **Optional decimation** (like `quad_decimate`) in a line buffer.
2. **Adaptive threshold:** per-tile min/max, dilated over neighboring tiles, using a few tile-rows of buffering.
3. **Streaming connected components:** single-pass union-find labeling for black and white regions, with an equivalence table that is resolved as rows complete.
4. **Boundary clustering:** pixels on black/white edges are grouped by their (black component, white component) pair. When both components close, the cluster is emitted. This is the irregular part; hashing and memory layout here decide feasibility.
5. **Quad fitting:** clusters go to fit engines (sort by angle, line-fit error, corner search). First implementation: run the fit on the RPU to get working end to end; then move the hot loop to PL fit engines if the RPU can't keep up.
6. **Decode and edge refinement:** by the time a quad is known, the tag's top rows have left the line buffers, so the decoder reads the needed pixels back from the DDR frame store. Reads are small and local (bit sampling grid plus edge strips), so this costs microseconds.
7. **Output:** tag ID, decision margin, Hamming distance and four subpixel corners in the TL/TR/BR/BL order the CPU code uses.

### Pose estimation (RPU)

- Port IPPE-square (single tag, two solutions) and SQPnP (multi-tag) from OpenCV to the RPU in C++, plus corner undistortion and tag angles as in [`TagAngleCalculator.cpp`](../src/cpp/estimator/TagAngleCalculator.cpp).
- The R5F has a double-precision FPU, so the math can match the golden model closely.
- Stereo: triangulate matched corners from the synced pair, then solve a joint PnP.
- Results go into a shared-memory ring that the APU publisher thread reads.

### MCL (PL + RPU)

- Particles are robot pose hypotheses. The motion model uses RobotData (heading, module positions) received by the APU and placed in shared memory.
- [PL] scoring engine: for each particle, project the expected tags (and later, map features) into each camera and score against detections. Many particles are scored in parallel per frame.
- [RPU] resampling and pose output.
- Mono input is manually triggered and stereo is continuous, per the diagram. The saved VSLAM map is an optional prior, loaded by the APU into shared memory.

### Object recognition and localization (DPU + APU)

- A small single-shot detector, trained on our game-piece datasets and quantized to INT8 with Vitis AI.
- The DPU is AMD's prebuilt accelerator IP in the PL; the APU loads the model and runs inference through the Vitis AI runtime.
- A PL downscaler on the FrameBus prepares the color stream, so the APU never resizes full frames.
- [APU] ground-plane projection of each box's bottom-center using camera extrinsics, published as ObjectData.

### TagSLAM (APU)

- Fixed-lag smoother over robot keyframes and tag landmarks, with reprojection factors from tag detections, odometry factors from RobotData and priors from the field layout, built on an existing factor-graph library.
- Publishes a robot pose estimate and, at low rate, a refined tag map (MapData). The refined map is saved to storage and can be loaded at boot.
- Runs as its own Linux process at normal priority, so it can never delay the tag path.

### VSLAM (APU, optional PL features)

- Built on an existing open-source visual SLAM system, adapted for our stereo pair and timestamps.
- Optional [PL] FAST corners and ORB descriptors to take feature extraction off the A53s.
- Map save/load gives the LocalGeometricMap → MapStorage → SavedVSLAMMap path in the diagram; the saved map is a prior for VSLAM and MCL.
- Mono VSLAM only when manually triggered.
- Treated as research. A failed attempt here doesn't block anything else. If the A53s are too slow, the documented fallback is the separate-CPU hardware option.

### IO and networking (APU)

- WPILib `ntcore` over the chip's Gigabit Ethernet, so NT4 details (time sync, reconnects, announce/subscribe) come from the maintained library.
- Config and field ingest reuse [`ConfigInterface.cpp`](../src/cpp/io/ConfigInterface.cpp) and [`FieldInterface.cpp`](../src/cpp/io/FieldInterface.cpp).
- A PREEMPT_RT publisher thread for the tag path; other publishers at normal priority.
- Hardware exposure timestamps are converted to NT server time using `ntcore`'s time-sync offset and sent as the value timestamp, as the CPU version does.
- Optional raw-UDP fast path from the RPU through a PL Ethernet MAC, only if Stage 5 shows it's needed.
- HTTP server for the setup-mode stream and a status page.

### Storage (APU)

- Files on eMMC or microSD: board ID, cached config, DPU model, saved tag and VSLAM maps.

## Interfaces and message formats

- **Pixel streams:** AXI-Stream, 8 bits per pixel, with frame start / line end sideband and a frame ID.
- **Control:** AXI-Lite registers per PL block, generated from one register description file and exposed to Linux (UIO or a small driver) and the RPU.
- **APU ↔ RPU:** OpenAMP/RPMsg for control, shared-memory rings for data.
- **FrameBus messages:** a versioned, fixed-layout binary format for everything above the pixel level: tag detections, poses, object boxes, features, RobotData, map updates. Each message carries the frame ID and exposure timestamp. The same format is used in every shared-memory ring, so blocks can move between PL, RPU and APU.

## Robot interface (NetworkTables contract)

The unit keeps the CPU pipeline's topics, so robot code doesn't change:

| Direction | Topic | Contents |
|---|---|---|
| Robot → unit | `/cameras/<hardware_id>/config/*` | `role`, `camera_matrix`, `distortion_coefficients`, `exposure`, `gain`, `width`, `height`, `compressed`, `setup_mode` |
| Robot → unit | `/field/tag_<id>` | 7 doubles per tag (x, y, z, qw, qx, qy, qz) |
| Unit → robot | `/cameras/<hardware_id>/output/observations` | Packed double array, same layout as [`OutputPublisher.cpp`](../src/cpp/io/OutputPublisher.cpp), with the exposure timestamp |
| Unit → robot | `/cameras/<hardware_id>/output/connected`, `capture_fps`, `processing_fps` | Status |

New outputs (object data, TagSLAM/MCL/VSLAM poses, refined map) get new topics under `/cameras/<hardware_id>/output/` or a unit-level table, defined in the stage that adds them. RobotData (heading, module positions) gets a new robot → unit topic in Stage 8.

The contract stays NT4-based so it works with both the roboRIO and SystemCore.

## Verification strategy

Each layer must pass before the next is trusted:

1. **Golden models** (`model/`): C++ models of each PL and RPU block, built from the CPU code. For integer blocks the model defines the exact expected output.
2. **Unit simulation:** cocotb + Verilator testbenches per PL block, compared with the golden model on synthetic and recorded frames. Integer blocks must match bit for bit; floating-point blocks must stay within a stated tolerance.
3. **Frame-replay simulation:** recorded match and practice-field videos (captured with the CPU system) replayed through the full simulated pipeline, comparing detections and poses with the CPU output.
4. **Hardware-in-the-loop:** recorded frames loaded into the DDR frame store from Linux and fed into the real PL pipeline in place of the sensor, so on-chip results can be compared frame by frame.
5. **Latency rig:** the LED glass-to-robot test from Stage 1, run on both systems.
6. **Load tests:** tag-path latency measured while the DPU, TagSLAM and VSLAM run flat out, to prove the tiers don't interfere.
7. **On-robot A/B:** CPU and FPGA units side by side on the practice robot, logging both to AdvantageKit/WPILOG.

## Risk register

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| Not enough HDL experience on the team | High | High | Custom hardware limited to the AprilTag and MCL blocks; training path (below); mentor review; keep the CPU system in competition |
| Effort spans several seasons and students graduate | High | High | Stage docs with exit criteria, golden models and tests as the "spec", pair new members with block owners |
| Quad fit / boundary clustering doesn't fit timing or RAM in PL | Medium | Medium | Run quad fit on the RPU first; PL fit engines later |
| Linux publish jitter hurts the tag path | Medium | Medium | PREEMPT_RT thread; hardware timestamps; raw-UDP fast path from the RPU if measured necessary |
| DPU, VSLAM and the frame store contend for DDR | Medium | Medium | Load tests in Stages 7 and 10; QoS settings on the memory interconnect; smaller DPU |
| A53 cores too slow for VSLAM | High | Low | Optional PL features; stretch goal only; separate-CPU hardware fallback |
| Vitis AI support for Zynq UltraScale+ changes or lapses | Medium | Medium | Check in Stage 0b; pin versions; CPU-only or NPU fallback for object detection |
| MIPI signal integrity on our own board | Medium | High | Stay on the dev kit through Stage 10; carrier board for the SOM, not a full chip-down design; vendor layout guides |
| FRC rule changes (power, cost, custom electronics) | Low | High | Check the game manual every season |
| Electrical noise and brownouts on the robot | Medium | High | Wide-input regulator, brownout-tolerant boot, watchdog, read-only root filesystem, status topic |

## FRC rules checklist

Check each item against the current season's game manual; rule numbers change every year.

- [ ] Custom circuits and coprocessors are allowed and connect to the control system only through allowed interfaces (Ethernet through the robot radio/switch).
- [ ] Powered from the PDH/PDP on an appropriately rated breaker or fuse.
- [ ] Total cost (module, carrier board, cameras) within the per-item cost limit and counted in the cost accounting.
- [ ] No wireless radios active on the board (disable any Wi-Fi/Bluetooth on dev kits).
- [ ] Any illumination LEDs follow the lighting rules.
- [ ] Mounting and enclosure meet the robot construction rules.

## Team and skills

- **Two tracks.** A small hardware track (SystemVerilog, simulation, Vivado) owns capture, AprilTag and MCL scoring. A larger software track (Linux C++, real-time firmware, ML) owns everything else, in tools most students already know.
- **Training path for the hardware track:** digital logic basics → HDL exercises (e.g. HDLBits) → Verilator + cocotb on a small block → owning a block from golden model to hardware.
- **Ownership:** each block has a student owner and a reviewer. Every RTL change needs a passing simulation and a review.
- **Keep it teachable:** short design notes per block in `docs/`, readable testbenches, and no "only one person understands this" blocks.

## Notes on the structure diagram

- [`GompeiVisionStructureDiagram.dot`](../GompeiVisionStructureDiagram.dot) uses `lhead=cluster3` on the `RobotState -> RobotData` edge, but there is no `cluster3` (the robot cluster is `cluster7`). Graphviz ignores it, so the render is fine, but it should probably be `cluster7` or removed.
- The diagram doesn't show where each node runs. The README's architecture section adds the PL / RPU / APU mapping.
