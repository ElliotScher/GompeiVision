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

- A self-contained, Limelight-sized **stereo module** (board, exactly 2 cameras, SSD and enclosure, a few inches on each side) that implements the macro architecture in [`GompeiVisionStructureDiagram.dot`](../GompeiVisionStructureDiagram.dot): capture, FrameBus, Mono/Stereo TagPnP, object recognition and localization, TagSLAM, stereo VIO, NetworkTables output, persistent map storage and onboard video recording. A robot carries as many modules as it needs.
- Exposure-to-measurement latency on the tag path that is low **and** nearly constant, with hardware exposure timestamps.
- Drop-in replacement for the CPU pipeline: robot code keeps using the same NetworkTables topics.
- Both of a module's cameras at 1280×800, ≥100 FPS for AprilTags, hardware-synchronized (to be confirmed in Stage 0b against the chosen part).
- Every module records both lenses to its own SSD, with per-frame timestamps that line up with robot logs.
- Fast recovery after a brownout or power cut: the tag path is producing measurements again within a few seconds, without waiting for Linux to boot.

**Non-goals**

- A general-purpose product for other teams. See the README's note to other teams.
- Putting everything in FPGA logic. Blocks go where they fit best (see [Where each block runs](#where-each-block-runs)).
- Writing our own NetworkTables client or neural-network accelerator when maintained ones exist.
- Replacing the CPU pipeline before the FPGA build reaches output parity (Stage 5).
- A custom ASIC. The processing tail is a fraction of a millisecond next to exposure and readout, so an ASIC would save almost nothing, costs far more and can't be changed after fabrication.

## Feasibility by block

Legend: **[PL]** FPGA logic, **[RPU]** real-time Cortex-R5F cores, **[APU]** Cortex-A53 cores running Linux.

| Diagram node | Implementation | Difficulty | Verdict and main risk |
|---|---|---|---|
| MonoCameras, StereoCameras | [PL] MIPI CSI-2 RX (vendor IP), hardware trigger and exposure-strobe timestamp counter; sensor I2C config from [APU] | Easy–moderate | Feasible. Risk: MIPI bring-up and signal integrity |
| MonoFrameBus, StereoFrameBus | [PL] AXI-Stream switch with per-consumer FIFOs and a DDR writer | Easy | Feasible |
| MonoTagPnP (detection) | [PL] streaming threshold → connected components with per-component feature accumulators → quad finding (corner scorecard with an on-demand contour tracer, or AprilTag 3's boundary clustering; chosen in Stage 1) → edge refinement → decode. See [Detectors and their roles](#detectors-and-their-roles) | Moderate–hard | Feasible. The core latency win, and the biggest piece of custom hardware. See [AprilTag detector](#apriltag-detector) |
| MonoTagPnP, StereoTagPnP (pose) | [RPU] IPPE-square and SQPnP, ported from the OpenCV calls in [`CameraPoseEstimator.cpp`](../src/cpp/estimator/CameraPoseEstimator.cpp) and [`SingleTagPoseEstimator.cpp`](../src/cpp/estimator/SingleTagPoseEstimator.cpp) | Easy–moderate | Feasible; expected well under 1 ms on an R5F with its double-precision FPU |
| TagAngleCalculator | [RPU] undistort corners, convert to angles | Easy | Feasible |
| Heading for single-tag disambiguation | Default: robot code picks between the two IPPE solutions using its own gyro history. Optional: heading-constrained solve on the [RPU], using the module's onboard IMU (required for VIO), timestamped by the [PL] | Easy (default) / moderate (IMU) | Feasible. Risk: heading offset handling and feedback loops. See [Heading and pose disambiguation](#heading-and-pose-disambiguation) |
| ObjectRecognition | [PL] AMD's prebuilt DPU accelerator, run from [APU] via Vitis AI, on the color module variant only | Moderate | Feasible. Risk: Vitis AI support and DPU size on the chosen part; model quantization accuracy |
| ObjectLocalization | [APU] ground-plane projection with camera extrinsics | Easy | Feasible |
| TagSLAM | [APU] fixed-lag sparse factor graph (tag landmarks, robot keyframes, odometry from RobotData) using an existing solver library | Moderate | Feasible; small problem (one field's tags). One instance per module, anchoring that module to the field |
| VIO | [PL] feature detection and tracking for both lenses + onboard IMU; [APU] lightweight VIO back end (filter or small sliding window) built on an existing open-source VIO system | Moderate–hard | Feasible. Much lighter than full SLAM because the tags are the map; estimate tens of Hz on the A53s (measured in Stage 9). Risk: defenders filling the view. See [Stereo VIO](#stereo-vio-pl--apu) |
| PoseMeasurements, ObjectData, MapData | [APU] WPILib `ntcore` in a real-time thread; optional fast path that skips Linux, driven by the [RPU]: raw UDP through a [PL] Ethernet MAC, or CAN-FD to SystemCore through a [PL] CAN-FD controller | Easy–moderate | Feasible. Risk: Linux publish jitter (measured in Stage 5) |
| MapStorage | [APU] refined tag map in files on eMMC | Easy | Feasible |
| Video recording (not in the diagram) | VCU hardware encoder (H.264/H.265) reading the DDR frame store; [APU] GStreamer writes segmented files to the module's M.2 SSD | Easy–moderate | Feasible. Both lenses at 120 FPS use about half the VCU's ~4K60 capacity. See [Video recording](#video-recording-vcu--apu) |
| Setup-mode stream (from CPU version) | [APU] MJPEG from the frame store, or H.264 from the VCU hardware encoder (the K26 has one) | Easy | Feasible |

**Overall:** feasible. The only large custom hardware block is the AprilTag detector (plus the smaller VIO feature tracker). Everything else is ported or existing software, or prebuilt IP.

## Where each block runs

The SoC FPGA has three kinds of compute sharing one DDR memory:

1. **FPGA logic (PL)** for anything that touches pixels at line rate or is massively parallel: capture, frame routing, AprilTag detection, VIO feature tracking, IMU sampling, and the DPU.
2. **Real-time cores (RPU)** for small, latency-critical numeric work that must not be delayed by Linux: PnP, tag angles, stereo PnP, CAN-FD messages.
3. **Linux CPU (APU)** for large, irregular, memory-hungry work and anything that benefits from existing libraries: `ntcore`, config, object-detection control, TagSLAM, the VIO back end, storage, recording, setup stream.

Rules:

- **The tag path never waits on ordinary Linux work.** Capture → detection → PnP runs entirely in PL and RPU. The finished measurement goes through shared memory to a high-priority real-time (PREEMPT_RT) publisher thread on the APU, which sends it with `ntcore`.
- **Measurement time comes from the hardware exposure timestamp**, converted to NT server time. Jitter in delivery never changes the timestamp the robot sees.
- **If Stage 5 measures publish jitter as too high**, add a fast path driven by the RPU that doesn't touch Linux at all: raw UDP through a small Ethernet MAC in PL, or CAN-FD on a dedicated SystemCore bus (see [CAN-FD fast path](#can-fd-fast-path)).
- **The tag path boots without Linux.** The first-stage boot loader loads the bitstream, the RPU firmware and the last config cached in flash, so capture → detection → PnP (and the fast path, if built) run within a few seconds of power-on. Linux attaches to the running RPU later (remoteproc attach, to be confirmed in Stage 2) and adds NetworkTables, config updates and everything else.
- All blocks exchange the versioned [FrameBus messages](#interfaces-and-message-formats) through shared memory, so a block can move between tiers without changes upstream or downstream.
- A block moves tiers only after a stage measures it missing its budget. The stage doc records the measurement and the decision.

### Multiple modules

Each module is independent: its own SoC, stereo pair, SSD, Ethernet and CAN-FD connections. Nothing on one module depends on another.

- **Per module:** capture, tags, PnP and stereo PnP, stereo VIO, TagSLAM, recording and, on the color variant, object detection.
- **The field frame is the shared language.** Each module's TagSLAM anchors its poses to the field using tags, so every module reports in field coordinates even though each module's VIO only tracks relative motion.
- **Fusion happens on the robot.** The robot's pose estimator fuses each module's measurements with their own timestamps and standard deviations, like running several Limelights. There's no single combined map.
- **Identity:** each module's board ID sets its NT topics (one `hardware_id` per lens, plus module-level outputs) and its CAN ID.
- **Shared CAN-FD bus:** each module sends about one 64-byte frame per camera frame, a few percent of a 5 Mbps bus, so several modules can share one dedicated vision bus.

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
| Sensor readout | overlaps with processing; only the rows *after* the last tag's bottom edge are waiting time (0 to almost a full readout, depending on where the tag is) |
| Streaming pipeline delay (threshold look-ahead, labeling) | 10–20 rows, ≈50–150 µs |
| Detection tail after the last tag row (component closes, quad fit, refine, decode) | 0.5–2 ms with boundary clustering, the RPU fit and DDR read-back; stretch target ≈20–200 µs with the corner scorecard and the on-chip strip buffer |
| PnP on RPU | <1 ms |
| Output | NT: hand-off to APU real-time thread + `ntcore` flush, <0.5 ms typical, jitter to be measured. CAN-FD fast path: ≈0.1–0.35 ms on the wire, near-constant |
| **Total exposure start → packet on the wire** | **~3–7 ms + remaining readout, with low jitter** (stretch: ~2–5 ms + remaining readout) |

### Readout is the floor

A global-shutter sensor exposes every row at the same moment, but still reads the rows out one at a time over MIPI. A tag near the bottom of the image waits for almost a full readout. Once the PL pipeline exists, exposure and readout dominate the budget, and the remaining levers are on the sensor side:

- **Sensor windowing:** read out only the rows where tags can appear. 1280×400 instead of 1280×800 roughly halves the worst-case readout.
- **More MIPI lanes or a faster lane rate.**
- **Short exposure,** with brighter tags or IR illumination (check the lighting rules).

This is also why the FPGA build uses MIPI rather than USB cameras (see [Sensors](#sensors)).

Object detection, TagSLAM and VIO aren't on this path. They have their own, looser budgets in their stage docs.

### What this buys on the robot

The robot loop runs every 20 ms (faster with high-frequency odometry threads), and pose estimators already compensate for latency using the measurement timestamp. So:

- The **biggest wins are jitter and timestamp accuracy.** A measurement whose age is known to ±0.1 ms fuses better than one known to ±5 ms, especially when the robot is turning fast.
- Lower absolute latency helps auto-align settle faster and reduces how far back the estimator has to replay odometry.
- After the packet leaves the unit, the robot-side NT server and the robot loop phase add up to one loop period. The fast paths make delivery shorter and steadier, but robot code still reads once per loop (see the [one-loop robot contract](#one-loop-robot-contract)); the exposure timestamps make that wait harmless for accuracy.
- At 120 FPS, 2–3 measurements arrive per 20 ms robot loop. Robot code must read **all** of them (NT `readQueue()` rather than `get()`, or every queued CAN frame), and fuse each with its own timestamp. Otherwise most of the extra frame rate is thrown away.

Stage 1 builds an **LED glass-to-robot rig** (an LED the robot toggles, seen by the camera, timestamped on both sides) so both systems are measured the same way.

## Bandwidth and memory budget

| Stream | Rate |
|---|---|
| One mono camera, 1280×800, 8-bit, 120 FPS | ≈123 MB/s |
| Stereo pair (one module) | ≈246 MB/s |
| Color camera for object detection, downscaled to e.g. 320×320 RGB at 30 FPS | ≈9 MB/s |
| VCU encoding both lenses (frame reads plus reference frames) | roughly 0.5–1 GB/s (estimate) |
| Encoded video to the SSD, both lenses, match mode | ≈1.5–3 MB/s (10–20 Mbps) |
| DDR frame store writes + DPU weights/activations + Linux + VIO | a few GB/s at most |

Zynq UltraScale+ modules use a wide DDR4 interface shared by PL, RPU and APU. Bandwidth is not expected to be the limit, but the DPU, VIO and the VCU compete with the frame store, so Stages 3, 7 and 9 measure memory contention and check that the tag path is unaffected. On-chip RAM (block RAM / UltraRAM) for line buffers, tile statistics, label tables and the DPU is the tighter budget; Stage 0b estimates it per block.

**On-chip strip buffer for decode read-back.** Refinement and decode only read pixels of tags that have just closed. So the PL can keep a rolling buffer of the last N rows in UltraRAM (for example 256 rows × 1280 px ≈ 320 KB) and read from it with fixed 1–2-cycle latency, instead of going to DDR. Tags taller than N rows fall back to the DDR frame store. This also keeps the tag path out of DDR contention with the DPU and VIO. A full on-chip double buffer (≈2 MB for two 1280×800 frames) would use most of the K26's UltraRAM, which the DPU also wants; Stage 0b checks the numbers.

## Hardware

### Selection criteria (Stage 0b)

1. SoC FPGA with Linux-capable application cores, real-time cores and FPGA logic sharing DDR.
2. MIPI CSI-2 receive for 2 cameras (one stereo pair per module).
3. Enough logic, DSP and on-chip RAM for both lenses' tag detection, VIO feature tracking, CAN-FD and, on the color variant, a DPU, with ≥30% headroom (see [Per-module resource estimate](#per-module-resource-estimate)).
4. Vitis AI (or equivalent) supports a DPU on the part.
5. An off-the-shelf module and dev kit available now, so we can prototype before designing our own board.
6. Covered by the free tier of the vendor toolchain.
7. Cost within FRC per-item limits.
8. A hardware video encoder (an "EV" Zynq UltraScale+ part; EG parts have none) and PCIe for an M.2 SSD.
9. Small enough for a Limelight-sized module, with heat the housing can remove.

### Candidates

| Option | Why | Concerns |
|---|---|---|
| **AMD Kria K26 SOM + dev kit (KV260 or KR260)** (primary) | Zynq UltraScale+ MPSoC with A53 + R5F + PL + DDR4 on one module; dev kits with camera connectors and Ethernet; vendor Ubuntu and PetaLinux images; Vitis AI DPU examples | Confirm current Vitis AI support level, free-tier coverage, MIPI port count on the kit and price. The module's footprint (≈77 × 60 mm) sets the minimum module size |
| Smaller Zynq UltraScale+ SOMs (ZU4EV class, roughly 4 × 5 cm) | Same chip family and toolchain; smaller, cooler, cheaper | Roughly 88k LUTs (check), too tight for a mono module's tags plus VIO features with headroom. Must be an EV part to keep the video encoder |
| Flash-based FPGA (e.g. Microchip PolarFire SoC, Lattice CrossLink-NX / Certus-NX) | Instant-on: the configuration survives power loss, so the logic is ready in milliseconds. Some Lattice parts have hardened MIPI receivers | Much less logic and RAM than a K26, no DPU, different toolchain. Considered for a future tag-only unit, not the primary |
| Newer SoC FPGAs (e.g. AMD Versal AI Edge Gen 2) | Much faster application cores (Cortex-A78AE class), which would remove the A53 bottleneck for VIO and TagSLAM | Availability in small modules, cost, toolchain maturity, power and whether there's a video encoder all need checking. Watch; re-check in Stage 0b |
| FPGA front end + faster CPU SoC (e.g. RK3588, or a Jetson Orin module) | Much faster CPU, an NPU, its own video encoder | Two chips, a CPU↔FPGA link (MIPI or PCIe) to design, no shared memory, and two chips' heat in a Limelight-sized housing (an Orin alone can add 10–25 W). Fallback only if Stage 9 shows the A53s can't run the VIO back end |

**Re-evaluation after dropping MCL and replacing VSLAM with VIO:** the K26 stays the primary choice. Dropping MCL frees its scoring engine (roughly 30–50k LUTs and 100–300 DSPs), and VIO needs far less CPU than full SLAM, so a mono module now fits on one K26 with the corner-scorecard detector (4B). The remaining pressure points are LUT headroom, A53 time for the VIO back end, and heat. The K26 is also about the largest module that fits a Limelight-sized housing, so bigger parts (ZU7EV class) aren't practical here. The faster-CPU options stay as fallbacks if Stage 9 measures the A53s as too slow.

All part-specific numbers (logic, DSP, RAM, MIPI ports, price) get filled in and verified in Stage 0b.

### Per-module resource estimate

Rough estimates, to be replaced by synthesis results in Stage 0b. The K26 has roughly 117k LUTs and 1,250 DSPs (check), so ≥30% headroom means using at most about 82k LUTs.

| Block | LUTs | DSPs | On-chip RAM |
|---|---|---|---|
| 2× MIPI RX, unpacking, timestamps, trigger | ~5–10k | 0 | small |
| Tag detection for both lenses, 4B (one pipeline shared between lenses, per-lens label state) | ~30–45k | ~30–60 | ~2–4 Mbit (incl. strip buffer) |
| VIO feature detection and tracking, both lenses | ~15–30k | ~20–60 | ~1–2 Mbit |
| IMU SPI, CAN-FD controller | ~3–6k | 0 | small |
| Interconnect, DMA, frame store, VCU interface | ~10–15k | 0 | small |
| **Mono module total** | **~65–105k** | **~50–120** | **~4–7 Mbit** |
| **Color variant** (small DPU instead of VIO features) | **~85–110k** | **~250–300** | **~6–10 Mbit** |

- The **mono module** fits the K26 but can break the 30% headroom rule at the high end. Ways to recover: use 4B rather than 4A, share one tag pipeline between the lenses (both lenses are ≈246 Mpx/s against ≈300 Mpx/s for one pixel per clock at 300 MHz), and keep the feature tracker lean.
- The **color variant** leaves VIO to the mono modules and spends that space on a small DPU. It's the tightest configuration.
- **A53 load per module (estimate):** VIO back end ~1–2 cores, TagSLAM ~1 core, NT and recording ~0.5 core. Stage 9 measures the VIO share.
- **Sensors:** VIO prefers wider lenses (more static features, harder for a defender to fill the view), while tag range prefers narrower ones. A higher-resolution mono global-shutter sensor would allow both, at the cost of more pixel throughput and recording at 60 FPS. Compare in Stage 0b.

### Sensors

- **AprilTags:** monochrome global shutter, 1–2 MP, ≥100 FPS, MIPI CSI-2, external trigger and strobe output (OV9281-class).
- **Objects:** a module variant whose pair is color global shutter (AR0234-class), run at lower FPS and downscaled for the DPU. Every module still has exactly two lenses.
- **Stereo:** every module's two sensors are identical, share one trigger line, and sit at the far ends of the module on a rigid mount. A Limelight-sized module allows roughly a 60–80 mm baseline. Depth error grows with distance squared: roughly 0.4 cm at 1 m, 3.5 cm at 3 m and 14 cm at 6 m (estimate for a ~70 mm baseline, ~70° lenses and quarter-pixel matching). Stereo is strongest up close; tag accuracy at range comes from PnP, not the baseline.
- **Why MIPI, not USB:** a USB (UVC) camera hands complete frames to Linux, and many have a bridge chip that buffers internally, so the PL can't start until the frame is done. MIPI CSI-2 sends uncompressed RAW8/RAW10 rows straight into the PL, one packet per row, with frame start/end markers. Leave the sensor's optional DPCM compression off.
- **Cable length:** MIPI flex cables are fragile and limited to roughly tens of centimeters. Inside a module the camera boards sit a few centimeters from the carrier on short, enclosed cables, so this mostly matters for dev-kit prototypes. Cameras mounted away from the unit use a serializer link (GMSL or FPD-Link over coax, a few µs of added delay). GigE Vision cameras are the fallback for off-the-shelf remote cameras: their packets carry rows as they're read out, and UDP is easy to parse in PL.

### Module form factor

About the size of a Limelight: roughly 3.5–4.5 × 2.5–3 × 1.5–2 in (estimate, set in Stage 10).

- **Stack, front to back:** camera boards at the far ends of the front plate → short MIPI flex → carrier board (M.2 2230 SSD underneath) → K26 SOM → heatsink and fan.
- **Back panel:** power, Ethernet, CAN-FD.
- **IMU:** on the carrier near the housing's mounting points and away from the fan, so it measures robot motion rather than board vibration. Required on every module for VIO.
- **Heat is the main constraint.** Estimated draw: K26 under full load ~8–15 W, cameras ~1 W, SSD writing ~2–3 W, so ~12–20 W in a few cubic inches. The aluminum housing is the heatsink, the SoC is thermally bonded to it, a small filtered fan moves air, and the chip temperature goes on the status topic.
- **Power:** 12 V through a buck-boost regulator, or PoE over the Ethernet cable for a single cable (check the FRC power rules).

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
| Golden models | C++ host build sharing code with [`src/`](../src/), the AprilTag C library, NAPA, OpenCV ArUco and ArUco Nano |
| CI | GitHub Actions: lint (Verible), Verilator + cocotb sims, golden-model and APU unit tests. Bitstream builds run nightly or on a self-hosted runner, since Vivado is large |
| Board and enclosure | KiCad, FreeCAD |

## Block design

### Capture

- MIPI CSI-2 RX from vendor IP, unpacked to 8-bit pixels on an AXI-Stream with start-of-frame and end-of-line markers.
- The PL drives each sensor's trigger input. A 64-bit free-running counter latches the exposure strobe; this is the measurement timestamp.
- Sensor setup (exposure, gain, resolution) over I2C from the APU, driven by `/cameras/<hardware_id>/config`.
- Each sensor has a fixed physical port, which fixes the "no serial number" problem we had with USB ThriftyCams: `hardware_id` comes from the port plus a board ID.

### FrameBus

- An AXI-Stream switch that fans each camera stream out to the consumers subscribed to it (tag pipeline, VIO feature tracker, downscaler for the DPU, DDR writer).
- Every frame is also written to a DDR ring buffer that PL, RPU and APU can all read. Consumers never wait for this copy; it's for decode and refinement read-back (below), object detection, setup-mode streaming and recording.
- Consumers that fall behind drop frames and count drops; they never back-pressure the sensor.

### Detectors and their roles

The FPGA detector is the only detector on the live tag path. Four software detectors support it.

| Detector | Runs on | Role |
|---|---|---|
| **FPGA streaming detector** ([below](#apriltag-detector)) | PL, plus RPU for repairs | The competition tag path: a tag finishes ~0.1 ms after the scan passes it, with fixed timing and no CPU use |
| **NAPA** ([zwatx/NAPA](https://github.com/zwatx/NAPA), GPLv3) | APU | First working tag path on the module (Stage 3), live fallback, and a 1-in-N background cross-check in matches. Also the source of the scorecard-style quad finding, ID lookup, refinement and shape repairs |
| **AprilTag 3 library** | Desktop (APU if needed) | Reference: golden model for threshold and blob labeling, and the parity baseline, since today's CPU pipeline uses it |
| **OpenCV ArUco** | APU in setup mode, and desktop | Camera and stereo calibration with ChArUco boards (Stage 6); multi-scale threshold comparison in Stage 1 |
| **ArUco Nano** | Desktop | Golden model for the contour tracer (4C); speed and accuracy comparison |

- **NAPA on the A53s.** Its published benchmarks (by the author and forum users, not ours) report ~0.5 ms per 1080p frame on one i7 core and ~0.7–1.6 ms on a Raspberry Pi 5–class board. A rough guess for the A53s is 2–6 ms per lens frame; Stage 3 measures it. NAPA waits for full frames, so it's roughly 5–15 ms slower than the streaming detector, and two lenses at 120 FPS would take about 0.5–1.5 cores that VIO and TagSLAM need. That's why it's the fallback, not the primary.
- **Fallback switch:** a config setting moves a module's tag path from the FPGA detector to NAPA without reflashing.
- **Cross-check:** NAPA runs on every Nth frame (for example 1 in 10) and its IDs and corners are compared with the FPGA's. Mismatches are counted on the status topic, which catches FPGA bugs that would otherwise go unnoticed.
- **Auto-labeling:** on the desktop, frames where AprilTag (full resolution), NAPA and ArUco agree become ground truth for Stage 1 datasets. Disagreements are flagged for a person; they're the hard cases worth keeping.
- **Shared weaknesses:** NAPA and the corner scorecard both need the tag's white quiet zone, and both lose corner accuracy when motion blur is longer than about one tag cell. Keep exposure short for shooting on the move.
- **Licensing:** NAPA is GPLv3, compatible with this repository's GPL-3.0.
- GPU detectors aren't considered, since the module has no GPU.

### AprilTag detector

The front end follows the AprilTag 3 algorithm used today ([`FiducialDetector.cpp`](../src/cpp/detector/FiducialDetector.cpp)), so threshold and blob labeling can be compared bit for bit. After that the design departs from it: quad finding and ID lookup follow NAPA, the robust fallback follows ArUco, and everything is rebuilt to stream.

| Stage | Origin |
|---|---|
| Threshold, blob labeling | AprilTag 3, streamed |
| Quad finding 4B (main path) | NAPA (extreme points) |
| Quad finding 4C (fallback) | ArUco (contour tracing) |
| Quad finding 4A (option) | AprilTag 3 (boundary clustering + line fit) |
| Refinement | AprilTag 3, with NAPA's outlier rejection |
| Bit sampling, brightness model | AprilTag 3 |
| ID lookup, error correction | NAPA |

Each block is tested bit for bit against a C++ model of the same block. The whole detector is tested by tolerance against AprilTag 3 at full resolution and NAPA: same IDs, corners within ~0.1 px, and a detection rate at least as good.

1. **Optional decimation** (like `quad_decimate`) in a line buffer. Full resolution costs no throughput in PL, so decimation only matters for label-table size.
2. **Adaptive threshold:** per-tile min/max, dilated over neighboring tiles, using a few tile-rows of buffering. If the look-ahead matters, an option is to use the previous frame's tile statistics (zero added rows, slight lag when lighting changes fast).
3. **Streaming connected components:** single-pass union-find labeling for black and white regions, with an equivalence table resolved during horizontal blanking, and label recycling. Each component carries a small **feature accumulator**, updated per pixel: area, bounding box and the extreme pixel in K directions (see 4B). Every field combines with a min, max or sum, so merging two components just merges their accumulators. A component **closes** when a row adds no pixels to it, and its accumulator is emitted right away, before the rest of the frame arrives.
4. **Quad finding: 4A, or 4B with the 4C tracer, chosen in Stage 1** on recorded data. Cheap filters on the accumulator run first (area, bounding-box aspect, and too dark overall, since every 36h11 code has at least 10 white cells), so most components never reach quad finding.
   - **4A. Boundary clustering + quad fit** (closest to the reference library). Pixels on black/white edges are grouped by their (black component, white component) pair. When both components close, the cluster goes to a fit engine: sort by angle, line-fit error from prefix sums, corner search. The first implementation runs the fit on the RPU; the hot loop moves to PL fit engines if the RPU can't keep up. This is the irregular part; hashing and memory layout decide feasibility.
   - **4B. Corner scorecard** (cheaper, fully streaming). For each black component, the accumulator tracks the pixel farthest along each of K = 16 directions (the max of x·cosθ + y·sinθ). Each corner of a convex quad is the extreme point for a range of directions equal to 180° minus its interior angle, so K = 16 catches corners up to about 157°. When the component closes, merge near-duplicate extremes and keep the 4 that enclose the largest area. No edge points are stored or sorted. It relies on the tag's white quiet zone (true for 36h11). Components that fail its sanity checks (blob much larger than its quad, implausible shape, too many separate extremes), which usually means a tag merged with dark background or partly hidden, go to 4C.
   - **4C. On-demand contour tracer** (ArUco-style, only for 4B rejects). A small PL block walks the rejected blob's outline in the on-chip strip buffer (or DDR, for blobs taller than the buffer), then runs the same angle sort, line fit and corner search as 4A. Estimate 10–50 µs per tag, with only one blob's points in memory at a time and roughly 8–15k LUTs. Capped at a few candidates per frame; extras are dropped and counted.
5. **Edge refinement:** only candidates that pass a quick decode on the rough quad are refined (NAPA's order), so text and texture are rejected before the expensive step. Sample across each of the 4 edges about every 1.5 px, find the edge (gradient centre for sharp edges, midpoint crossing for blurred ones), drop outlier samples so a short obstruction doesn't drag a corner, least-squares fit each edge and intersect. This produces the subpixel corners for 4B and 4C, and improves 4A.
6. **Decode:** closed-form square-to-quad homography (Heckbert's projective mapping: a few dozen multiplies and two divides, no linear solve). Sample the border and data cells, fit the black/white brightness model from the border, and threshold each bit. Look up the ID NAPA's way: any code within 3 bit errors shares one of the payload's four 9-bit blocks exactly, so four lookups in 512-entry block-RAM tables narrow the 2,348 code/rotation candidates to a handful, which are then checked exactly. That takes a few clock cycles. A third bit error is allowed only when the wrong cells touch (glare, a finger), so random patterns don't match by chance.
7. **Pixel read-back:** by the time a tag closes, its top rows have left the line buffers. Refinement and decode read from the on-chip strip buffer (see [Bandwidth and memory budget](#bandwidth-and-memory-budget)), falling back to the DDR frame store. Reads are small and local (bit sampling grid plus edge strips), so this costs microseconds.
8. **Frame boundaries:** every block resets its per-frame state (labels, accumulators, tile statistics) when the start-of-frame sideband reaches it, so blocks can be working on different frames at the same time. Every result carries the frame ID. A frame-store slot isn't reused until all of its tags are decoded; if it would be, the remaining candidates are dropped and counted, never stalled.
9. **Output:** tag ID, decision margin, Hamming distance and four subpixel corners in the TL/TR/BR/BL order the CPU code uses.

Fixed-point widths for the homography and refinement come from the golden model's error budget (target: corners within 0.1 px of the CPU output).

**Shape repairs (from NAPA, on the RPU):** for the few tags that need them, the RPU tries bowed sides (wide lenses), sides cut by the frame edge, a grey quiet zone, a side one cell out, clipped exposure, and blur/gamma shifts. Each repair is kept only if the tag still decodes to the same ID.

### Pose estimation (RPU)

- Port IPPE-square (single tag, two solutions) and SQPnP (multi-tag) from OpenCV to the RPU in C++, plus corner undistortion and tag angles as in [`TagAngleCalculator.cpp`](../src/cpp/estimator/TagAngleCalculator.cpp).
- The R5F has a double-precision FPU, so the math can match the golden model closely.
- Stereo: triangulate matched corners from the synced pair, then solve a joint PnP.
- Results go into a shared-memory ring that the APU publisher thread reads.

### Heading and pose disambiguation

Single-tag PnP has two solutions. Using the robot's heading to pick between them, or to constrain the solve, is the idea behind MegaTag2-style localization. Options, in order:

1. **Default: disambiguate on the robot.** The unit already publishes both IPPE solutions and their errors. Robot code looks up its gyro heading at the measurement timestamp in the pose estimator's history and picks the matching solution. No heading goes back to the unit, so nothing waits on a round trip.
2. **Heading-constrained solve on the RPU,** if we want the unit to output a final pose. The RPU needs the heading *at exposure time*. It never buffers frames waiting for heading: it keeps a short heading history and interpolates, or extrapolates from the latest heading plus yaw rate (the error is about ½ × angular acceleration × gap², negligible for gaps of a few ms). Heading and yaw rate come from RobotData (Stage 8), sent every robot loop.
3. **Onboard IMU** (on every module, since VIO needs it): a MEMS gyro and accelerometer on SPI, sampled at ≥1 kHz and timestamped by the same PL counter as the camera exposures. A complementary or small Kalman filter corrects its drift and field-zero offset using:
   - the robot's heading from RobotData (slow correction, and the field zero), and
   - **unconstrained** multi-tag PnP heading, only when quality gates pass (≥2 tags, low reprojection error, tags close and large, slow rotation).

   Gate corrections on measurement quality, not on whether tags are in view. Never feed the heading-constrained solve's own output back into the filter: that's circular and lets errors reinforce.
4. **Listening to the robot's IMU on CAN:** CAN is broadcast, so a unit on the IMU's bus could timestamp the IMU's own frames. Commercial FRC IMU protocols are generally undocumented and robot code keeps its own heading offset, so this is only worth it with an IMU whose protocol we control.

### Object recognition and localization (DPU + APU)

- A small single-shot detector, trained on our game-piece datasets and quantized to INT8 with Vitis AI.
- The DPU is AMD's prebuilt accelerator IP in the PL; the APU loads the model and runs inference through the Vitis AI runtime.
- A PL downscaler on the FrameBus prepares the color stream, so the APU never resizes full frames.
- [APU] ground-plane projection of each box's bottom-center using camera extrinsics, published as ObjectData.

### TagSLAM (APU)

- Fixed-lag smoother over robot keyframes and tag landmarks, with reprojection factors from tag detections, odometry factors from RobotData and priors from the field layout, built on an existing factor-graph library.
- Publishes a robot pose estimate and, at low rate, a refined tag map (MapData). The refined map is saved to storage and can be loaded at boot.
- Runs as its own Linux process at normal priority, so it can never delay the tag path.

### Stereo VIO (PL + APU)

Visual-inertial odometry, not full SLAM. The tags are the map, so VIO only has to bridge the gaps between tag sightings (a few seconds under defense) with accurate position change and velocity, where wheel odometry fails because of slip and pushing.

- [PL] feature detection (FAST-style corners) and frame-to-frame tracking (KLT-style) for both lenses at full frame rate, plus stereo matches for depth, so VIO has true scale from the start.
- [PL] onboard IMU sampled at ≥1 kHz and timestamped by the same counter as the camera exposures.
- [APU] lightweight back end (an MSCKF-style filter or a small sliding window) built on an existing open-source VIO system, fed tracked features, depth and IMU data.
- Outlier rejection for moving objects: features whose motion disagrees with the IMU, and very close features during contact (likely a defender), are down-weighted.
- Publishes pose deltas and velocity per frame with exposure timestamps. Robot code fuses them with tag measurements, and VIO velocity feeds shoot-on-the-move.
- 120 FPS keeps motion between frames small, so tracking survives the jolts of contact better than at 30–60 FPS.
- Benchmarked against QuestNav (a Meta Quest headset used as a VIO sensor) in Stage 9. Full VSLAM (mapping, loop closure) is out of scope.

### IO and networking (APU)

- WPILib `ntcore` over the chip's Gigabit Ethernet, so NT4 details (time sync, reconnects, announce/subscribe) come from the maintained library.
- Config and field ingest reuse [`ConfigInterface.cpp`](../src/cpp/io/ConfigInterface.cpp) and [`FieldInterface.cpp`](../src/cpp/io/FieldInterface.cpp).
- A PREEMPT_RT publisher thread for the tag path; other publishers at normal priority.
- Hardware exposure timestamps are converted to NT server time using `ntcore`'s time-sync offset and sent as the value timestamp, as the CPU version does.
- Optional fast path, only if Stage 5 shows it's needed: raw UDP from the RPU through a PL Ethernet MAC, or CAN-FD to SystemCore (below).
- HTTP server for the setup-mode stream and a status page.

### CAN-FD fast path

Optional, and SystemCore only: SystemCore has CAN-FD buses, while the roboRIO's CAN is classic CAN 2.0 (8-byte frames at 1 Mbps). NT stays the contract either way.

- **Dedicated bus:** the modules and SystemCore alone on one of SystemCore's buses, terminated with 120 Ω at both ends. No motor traffic to wait behind, and a faulty module can't disturb drivetrain control. Each module uses only a few percent of the bus and has its own CAN ID from its board ID.
- **Controller:** the Zynq's built-in CAN controllers are classic CAN only. Use a CAN-FD controller in PL (AMD's CAN FD IP, check its license, or an open-source core) driven by the RPU, or an external MCP2518FD over SPI (adds tens of µs). The transceiver must be CAN-FD rated (TJA1462, TCAN1042 or MCP2562FD class).
- **Timing:** a 64-byte frame takes about 0.35 ms at a 2 Mbps data rate, 0.16 ms at 5 Mbps and 0.11 ms at 8 Mbps, with near-constant transit on a dedicated bus.
- **IDs:** FRC extended IDs with manufacturer "Team Use" and device type "Miscellaneous", read on the robot through WPILib's `CAN` class. Check how the SystemCore API exposes FD frames and bus selection.
- **No clock sync needed:** each frame carries the measurement's *age* at send time, and robot code computes `capture time = receive timestamp − age − fixed transit`.
- **Robot side:** drained once per loop under the [one-loop robot contract](#one-loop-robot-contract). WPILib's custom-device `CAN` class keeps only the newest packet per message ID, so the module rotates the API index with the frame counter (for example, frame counter mod 8) and robot code reads all 8 slots each loop, keeping the new ones. Check whether SystemCore's API offers a real receive queue instead.
- Because the RPU drives it, this path works before Linux has finished booting.

The draft frame layout is in [Interfaces and message formats](#interfaces-and-message-formats).

### Video recording (VCU + APU)

- The VCU (the Zynq's hard H.264/H.265 encoder) reads both lenses from the DDR frame store. Its total capacity is about 4K at 60 FPS (≈500 Mpx/s). Both lenses at 1280×800 and 120 FPS are ≈246 Mpx/s, about half, leaving room for the setup stream. Recording at 30–60 FPS is enough for match replays and leaves more headroom.
- The two lenses are encoded **side by side** in one 2560×800 stream, so left and right frames stay paired in one file.
- Mono sensors: encode as monochrome (4:0:0) if the VCU and our tool version support it; otherwise fill the chroma planes with a constant in PL.
- [APU] GStreamer with AMD's VCU plugins writes **segmented** files (a few seconds each, fragmented MP4 or MKV) to an ext4 or f2fs data partition on the M.2 SSD, separate from the read-only root. A power cut loses at most the last segment.
- A **metadata file** per recording: frame ID, hardware exposure timestamp, detections and poses for each frame, and a sync point to the robot's AdvantageKit/WPILOG log.
- Two modes: **match** (H.265, 30–60 FPS, ≈10–20 Mbps per module, ≈0.2–0.4 GB per match) and **dataset** (raw frames, ≈246 MB/s per module, for golden-model datasets and VIO replays; practice sessions only, since compression changes pixel values).
- Started and stopped from robot code over NT (for example, start at match start).
- Recording never touches the tag path; Stage 3's contention test confirms it.

### Storage (APU)

- Files on eMMC: board ID, cached config, DPU model, the saved refined tag map. No SD card on the robot (they work loose and corrupt on power cuts); the root filesystem is read-only. Video and dataset recordings go to the M.2 SSD.

## Interfaces and message formats

- **Pixel streams:** AXI-Stream, 8 bits per pixel, with frame start / line end sideband and a frame ID.
- **Control:** AXI-Lite registers per PL block, generated from one register description file and exposed to Linux (UIO or a small driver) and the RPU.
- **APU ↔ RPU:** OpenAMP/RPMsg for control, shared-memory rings for data.
- **FrameBus messages:** a versioned, fixed-layout binary format for everything above the pixel level: tag detections, poses, object boxes, features, RobotData, map updates. Each message carries the frame ID and exposure timestamp. The same format is used in every shared-memory ring, so blocks can move between PL, RPU and APU.
- **CAN-FD frame (draft, fast path only):** the same content as `observations`, quantized into one 64-byte frame.

  | Bytes | Field |
  |---|---|
  | 0–1 | Frame counter (diagnostics and CAN slot rotation; robot code doesn't need to remember it) |
  | 2–3 | Age at send time, µs (up to 65 ms) |
  | 4 | Flags: camera index, number of solutions (1 or 2), format version |
  | 5 | Tag count (up to 8) |
  | 6–7 | Reserved |
  | 8–23 | Solution 0: X, Y, Z (int16, mm), quaternion W, X, Y, Z (int16, scaled by 1/32767), error (uint16) |
  | 24–39 | Solution 1, same layout (zero if absent) |
  | 40–47 | Tag IDs (uint8 each) |
  | 48–63 | Tag distances (uint16, mm each) |

## Robot interface (NetworkTables contract)

The unit keeps the CPU pipeline's topics, so robot code doesn't change:

| Direction | Topic | Contents |
|---|---|---|
| Robot → unit | `/cameras/<hardware_id>/config/*` | `role`, `camera_matrix`, `distortion_coefficients`, `exposure`, `gain`, `width`, `height`, `compressed`, `setup_mode` |
| Robot → unit | `/field/tag_<id>` | 7 doubles per tag (x, y, z, qw, qx, qy, qz) |
| Unit → robot | `/cameras/<hardware_id>/output/observations` | Packed double array, same layout as [`OutputPublisher.cpp`](../src/cpp/io/OutputPublisher.cpp), with the exposure timestamp |
| Unit → robot | `/cameras/<hardware_id>/output/connected`, `capture_fps`, `processing_fps` | Status |

New outputs (object data, TagSLAM poses, VIO pose deltas and velocity, refined map) get new topics under `/cameras/<hardware_id>/output/` or a unit-level table, defined in the stage that adds them. RobotData (heading, module positions) gets a new robot → unit topic in Stage 8.

The contract stays NT4-based so it works with both the roboRIO and SystemCore. Robot code should read `observations` with a queue (`readQueue()`), since several measurements arrive per robot loop at 120 FPS.

Each module publishes per-lens topics under its lenses' `hardware_id`s and module-level outputs (stereo, TagSLAM and VIO outputs, recording status) under its board ID. Recording start/stop is a new robot → unit topic, defined in Stage 3.

### One-loop robot contract

Everything the robot controller receives from the modules is fully handled within the loop it's read in. The only vision state that lives longer is the pose estimator's own history.

- **Each message is self-contained.** One NT value or one CAN-FD frame carries everything for one camera frame: capture timing, pose solutions, tag IDs and quality. Nothing is split across messages that would need reassembly.
- **Absolute time, computed on arrival.** NT values carry the exposure time in NT server time. CAN-FD frames carry an age that robot code turns into an absolute timestamp using the receive time. No clock-sync state in robot code.
- **Drain, use, discard.** Each loop: read every queued message (NT `readQueue()`, every CAN slot), compute capture times, pick between single-tag solutions using the pose estimator's history at the capture time (`sampleAt()`), reject bad measurements against the current estimate, set standard deviations, call `addVisionMeasurement()`, then drop the messages.
- **State that would need to persist lives on the module:** drop and duplicate counting, consistency checks across frames, camera health and timeouts. Modules publish these as status, which robot code reads each loop.
- **One path per measurement.** Sending the same measurement over both CAN-FD and NT would need deduplication across loops, so each module sends measurements over one path (CAN-FD when available) and uses NT for everything else.
- **Budget:** 4 modules × 2 lenses × 120 FPS is about 20 measurements per 20 ms loop, or about 10 if each module sends one stereo measurement per frame. Each `addVisionMeasurement()` replays odometry from its capture time, so Stage 5 measures the per-loop cost.

## Verification strategy

Each layer must pass before the next is trusted:

1. **Golden models** (`model/`): C++ models of each PL and RPU block, built from the CPU code, NAPA and ArUco Nano (see [Detectors and their roles](#detectors-and-their-roles)). For integer blocks the model defines the exact expected output.
2. **Unit simulation:** cocotb + Verilator testbenches per PL block, compared with the golden model on synthetic and recorded frames. Integer blocks must match bit for bit; floating-point blocks must stay within a stated tolerance.
3. **Frame-replay simulation:** recorded match and practice-field videos (captured with the CPU system) replayed through the full simulated pipeline, comparing detections and poses with the CPU output.
4. **Hardware-in-the-loop:** recorded frames loaded into the DDR frame store from Linux and fed into the real PL pipeline in place of the sensor, so on-chip results can be compared frame by frame.
5. **Latency rig:** the LED glass-to-robot test from Stage 1, run on both systems.
6. **Load tests:** tag-path latency measured while the DPU, TagSLAM, VIO and recording run flat out, to prove the tiers don't interfere.
7. **On-robot A/B:** CPU and FPGA units side by side on the practice robot, logging both to AdvantageKit/WPILOG.
8. **Power-cut recovery:** cut power mid-stream and measure how long until the tag path (and fast path, if built) produces measurements again, and until NT is back.
9. **Thermal:** a closed module on a hot robot, with every feature and recording running for a full practice day, chip temperature logged.

## Risk register

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| Not enough HDL experience on the team | High | High | Custom hardware limited to the AprilTag and VIO feature-tracking blocks; training path (below); mentor review; keep the CPU system in competition |
| Effort spans several seasons and students graduate | High | High | Stage docs with exit criteria, golden models and tests as the "spec", pair new members with block owners |
| Quad fit / boundary clustering doesn't fit timing or RAM in PL | Medium | Medium | Run quad fit on the RPU first; PL fit engines later |
| Linux publish jitter hurts the tag path | Medium | Medium | PREEMPT_RT thread; hardware timestamps; raw-UDP fast path from the RPU if measured necessary |
| DPU, VIO and the frame store contend for DDR | Medium | Medium | Load tests in Stages 7 and 9; QoS settings on the memory interconnect; smaller DPU |
| A53 cores too slow for the VIO back end | Medium | High | PL feature tracking; lightweight filter back end; faster-CPU hardware fallback (see [Candidates](#candidates)) |
| A defender fills the camera view and corrupts VIO | High | Medium | Modules facing different directions, IMU-based outlier rejection, stereo depth; optional upward-angled module |
| Vitis AI support for Zynq UltraScale+ changes or lapses | Medium | Medium | Check in Stage 0b; pin versions; CPU-only or NPU fallback for object detection |
| MIPI signal integrity on our own board | Medium | High | Stay on the dev kit through Stage 9; carrier board for the SOM, not a full chip-down design; vendor layout guides |
| FRC rule changes (power, cost, custom electronics) | Low | High | Check the game manual every season |
| Electrical noise and brownouts on the robot | Medium | High | Buck-boost regulator that holds the input steady through ~7 V sags, brownout-tolerant boot, watchdog, read-only root filesystem, eMMC boot (no SD card), status topic |
| Unit reboots mid-match (hit or brownout) and is blind while Linux boots | Medium | High | Tag path boots from the first-stage loader without Linux; fast path from the RPU; recovery time measured in Stage 10 |
| MIPI flex cable cracks or works loose on the robot | Medium | High | Locked mounting and strain relief, short runs, spares; serializer links (GMSL/FPD-Link) for cameras away from the unit |
| Corner scorecard (4B) misses tags that touch dark background or are occluded | Medium | Low | Measured in Stage 1; on-demand contour tracer (4C); or use 4A |
| FPGA detector bug or bad bitstream at an event | Low | High | NAPA fallback by config switch; 1-in-N cross-check catches silent errors |
| SystemCore's CAN-FD API or bus setup doesn't suit a custom device | Medium | Low | NT stays the contract; raw UDP as the alternative fast path |
| Heading filter corrected by its own constrained-solve output (feedback loop) | Low | Medium | Only unconstrained, quality-gated multi-tag headings feed the filter |
| Module overheats in a Limelight-sized housing (~12–20 W) | Medium | High | Housing as heatsink, filtered fan, temperature on the status topic, thermal test in Stage 10; lower VIO rate or recording FPS when hot |
| Short stereo baseline limits depth accuracy at range | High | Low | Lenses at the module's far ends; stereo up close, PnP for tags at range |
| SSD corruption on power cuts | Medium | Low | Segmented recording, separate data partition, journaling filesystem |

## FRC rules checklist

Check each item against the current season's game manual; rule numbers change every year.

- [ ] Custom circuits and coprocessors are allowed and connect to the control system only through allowed interfaces (Ethernet through the robot radio/switch; CAN, if the CAN-FD fast path is built).
- [ ] If on CAN: custom CAN devices are allowed, use the "Team Use" manufacturer and "Miscellaneous" device type IDs, and bus wiring and termination follow the rules.
- [ ] Frame grounding and ESD requirements are met for the unit and its enclosure.
- [ ] Powered from the PDH/PDP on an appropriately rated breaker or fuse (if using PoE, the injector and wiring are allowed).
- [ ] Each module's cost (SOM, carrier board, two cameras, SSD, housing) within the per-item cost limit, and every module counted in the cost accounting.
- [ ] No wireless radios active on the board (disable any Wi-Fi/Bluetooth on dev kits).
- [ ] Any illumination LEDs follow the lighting rules.
- [ ] Mounting and enclosure meet the robot construction rules.

## Team and skills

- **Two tracks.** A small hardware track (SystemVerilog, simulation, Vivado) owns capture, AprilTag and VIO feature tracking. A larger software track (Linux C++, real-time firmware, ML) owns everything else, in tools most students already know.
- **Training path for the hardware track:** digital logic basics → HDL exercises (e.g. HDLBits) → Verilator + cocotb on a small block → owning a block from golden model to hardware.
- **Ownership:** each block has a student owner and a reviewer. Every RTL change needs a passing simulation and a review.
- **Keep it teachable:** short design notes per block in `docs/`, readable testbenches, and no "only one person understands this" blocks.

## Notes on the structure diagram

- [`GompeiVisionStructureDiagram.dot`](../GompeiVisionStructureDiagram.dot) shows one module: a stereo pair and the onboard IMU, with MCL removed, VSLAM replaced by VIO, and video recording added. The old `lhead=cluster3` attribute, which pointed at a cluster that didn't exist, is gone.
- The diagram doesn't show where each node runs. The README's architecture section adds the PL / RPU / APU mapping.
