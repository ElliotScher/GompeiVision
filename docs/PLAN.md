# GompeiVision FPGA plan

This document covers whether an FPGA-accelerated GompeiVision is feasible, how each block would be built, and how we'll know it works. The [README](../README.md) has the summary; per-stage specs are in [stages/](stages/README.md).

Every latency, resource, timing and cost figure here is an **estimate**. Figures become facts only when a stage measures them, and the stage doc records the result.

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
10. [Robot interface](#robot-interface)
11. [Verification strategy](#verification-strategy)
12. [Risk register](#risk-register)
13. [FRC rules checklist](#frc-rules-checklist)
14. [Team and skills](#team-and-skills)
15. [Notes on the structure diagram](#notes-on-the-structure-diagram)

## Goals and non-goals

**Goals**

- One product: a self-contained, Limelight-sized **single-lens module** (K26 SOM, one global-shutter camera, SSD and a machined housing, a few inches on each side), **as cheap as we can make it**. Target: roughly $400–580 per module (see [Module cost](#module-cost)). A robot carries as many modules as it needs.
- Every module does streaming AprilTag detection and **built-in object detection** (the DPU in the FPGA logic), records its own video, and implements the macro architecture in [`GompeiVisionStructureDiagram.dot`](../GompeiVisionStructureDiagram.dot).
- Exposure-to-measurement latency on the tag path that is low **and** nearly constant, with hardware exposure timestamps.
- **Tag corners go to the robot over CAN-FD, and PnP runs on the SystemCore** (see [Robot-side PnP](#robot-side-pnp)). One solve can use every module's corners, the live gyro, and the tag map and camera positions that robot code already owns.
- Drop-in compatibility with the CPU pipeline: the module also solves PnP itself and publishes the same NetworkTables topics, for dashboards, debugging, roboRIO robots and teams not using our robot library.
- 1280×800 at ≥100 FPS for AprilTags (to be confirmed in Stage 0b against the chosen sensor).
- Every module records to its own SSD, with per-frame timestamps that line up with robot logs.
- Fast recovery after a brownout or power cut: the tag path is producing corners again within a few seconds, without waiting for Linux to boot.

**Non-goals**

- A general-purpose product for other teams. See the README's note to other teams.
- Putting everything in FPGA logic. Blocks go where they fit best (see [Where each block runs](#where-each-block-runs)).
- Writing our own NetworkTables client or neural-network accelerator when maintained ones exist.
- Stereo, VIO or full VSLAM. With one lens per module, localization between tag sightings comes from wheel odometry and the gyro. FRC fields have enough tags that this is the best value for the cost.
- Replacing the CPU pipeline before the FPGA build reaches output parity (Stage 5).
- A custom ASIC. The processing tail is a fraction of a millisecond next to exposure and readout, so an ASIC would save almost nothing, costs far more and can't be changed after fabrication.

## Feasibility by block

Legend: **[PL]** FPGA logic, **[RPU]** real-time Cortex-R5F cores, **[APU]** Cortex-A53 cores running Linux, **[Robot]** robot code on the SystemCore.

| Diagram node | Implementation | Difficulty | Verdict and main risk |
|---|---|---|---|
| Camera | [PL] MIPI CSI-2 RX (vendor IP), hardware trigger and exposure-strobe timestamp counter; sensor I2C config from [APU] | Easy–moderate | Feasible. Risk: MIPI bring-up and signal integrity |
| FrameBus | [PL] AXI-Stream switch with per-consumer FIFOs and a DDR writer | Easy | Feasible |
| TagDetection | [PL] streaming threshold → connected components with per-component feature accumulators → quad finding (corner scorecard with an on-demand contour tracer, or AprilTag 3's boundary clustering; chosen in Stage 1) → edge refinement → decode. See [Detectors and their roles](#detectors-and-their-roles) | Moderate–hard | Feasible. The core latency win, and the biggest piece of custom hardware. See [AprilTag detector](#apriltag-detector) |
| CornerOutput | [RPU] undistorts corners with the camera calibration and sends them as CAN-FD frames through a [PL] CAN-FD controller | Easy–moderate | Feasible. Risk: SystemCore's API for custom CAN-FD devices. See [CAN-FD tag path](#can-fd-tag-path) |
| Robot-side PnP | [Robot] IPPE-square per tag, multi-tag refinement across all modules, optional heading-constrained solve; in a small robot library (Java first, C++ over JNI if measured necessary) | Easy–moderate | Feasible; tens of µs per tag and roughly 0.1–1 ms per loop on the SystemCore's Cortex-A76 cores. See [Robot-side PnP](#robot-side-pnp) |
| Module-side PnP | [RPU] IPPE-square and SQPnP, ported from the OpenCV calls in [`CameraPoseEstimator.cpp`](../src/cpp/estimator/CameraPoseEstimator.cpp) and [`SingleTagPoseEstimator.cpp`](../src/cpp/estimator/SingleTagPoseEstimator.cpp), published over NT only | Easy–moderate | Feasible; expected well under 1 ms on an R5F with its double-precision FPU |
| TagAngleCalculator | [RPU] undistort corners, convert to angles | Easy | Feasible |
| ObjectRecognition | [PL] AMD's prebuilt DPU accelerator, run from [APU] via Vitis AI, on every module | Moderate | Feasible. Risk: Vitis AI support and how large a DPU fits next to the tag pipeline; model quantization accuracy |
| ObjectLocalization | [APU] ground-plane projection with camera extrinsics | Easy | Feasible. Single-camera range is only good for pieces on the floor |
| TagSLAM | Desktop tool (optionally [APU] later): offline factor graph over logged corners from all modules plus robot odometry, used as field calibration | Moderate | Feasible; small problem (one field's tags), no real-time budget. See [TagSLAM](#tagslam-offline-field-calibration) |
| PoseMeasurements, ObjectData, MapData | CAN-FD corner frames from the [RPU] (primary tag path); [APU] WPILib `ntcore` in a real-time thread for module-side poses, objects, status and everything else | Easy–moderate | Feasible. Risk: Linux publish jitter on the NT path (measured in Stage 5) |
| MapStorage | [APU] refined tag map in files on eMMC | Easy | Feasible |
| Video recording (not in the diagram) | VCU hardware encoder (H.264/H.265) reading the DDR frame store; [APU] GStreamer writes segmented files to the module's M.2 SSD | Easy–moderate | Feasible. One lens at 120 FPS uses about a quarter of the VCU's ~4K60 capacity. See [Video recording](#video-recording-vcu--apu) |
| Setup-mode stream (from CPU version) | [APU] MJPEG from the frame store, or H.264 from the VCU | Easy | Feasible |

**Overall:** feasible. The only large custom hardware block is the AprilTag detector. Everything else is ported or existing software, or prebuilt IP.

## Where each block runs

The SoC FPGA has three kinds of compute sharing one DDR memory, and the robot controller adds a fourth:

1. **FPGA logic (PL)** for anything that touches pixels at line rate or is massively parallel: capture, frame routing, AprilTag detection, the downscaler and the DPU, and the CAN-FD controller.
2. **Real-time cores (RPU)** for small, latency-critical work that must not be delayed by Linux: corner undistortion, CAN-FD corner frames, module-side PnP and tag angles.
3. **Linux CPU (APU)** for large, irregular, memory-hungry work and anything that benefits from existing libraries: `ntcore`, config, object-detection control and localization, storage, recording, setup stream.
4. **Robot controller (SystemCore)** for anything that needs robot state: PnP over all modules' corners, heading-constrained solves, and pose estimation.

Rules:

- **The tag path never waits on Linux.** Capture → detection → corner undistortion → CAN-FD runs entirely in PL and RPU.
- **Measurement time comes from the hardware exposure timestamp.** CAN-FD frames carry the measurement's age at send time; NT values carry the exposure time in NT server time. Jitter in delivery never changes the capture time the robot sees.
- **Pose solving needs robot state, so it runs on the robot.** The robot owns the gyro, the tag map, every module's mounting position and the pose estimator. The module sends what only it has: corners, quality and timing.
- **The tag path boots without Linux.** The first-stage boot loader loads the bitstream, the RPU firmware and the last config cached in flash, so capture → detection → CAN-FD runs within a few seconds of power-on. Linux attaches to the running RPU later (remoteproc attach, to be confirmed in Stage 2) and adds NetworkTables, object detection, config updates and everything else.
- All blocks exchange the versioned [FrameBus messages](#interfaces-and-message-formats) through shared memory, so a block can move between tiers without changes upstream or downstream.
- A block moves tiers only after a stage measures it missing its budget. The stage doc records the measurement and the decision.

### Multiple modules

Each module is independent: its own SoC, camera, SSD, Ethernet and CAN-FD connections. Nothing on one module depends on another.

- **Per module:** capture, tag detection, corner output, module-side PnP for NT, object detection, recording.
- **Fusion happens on the robot.** Robot code gets every module's corners in the same loop. It can solve each camera frame separately and fuse the results in the pose estimator, or run one joint solve across modules (see [Robot-side PnP](#robot-side-pnp)).
- **Optional shared trigger.** Modules expose at different moments unless they share a trigger line, and at 5 m/s the robot moves up to ~5 cm in one 10 ms loop. A joint solve across modules needs either a shared trigger (one module configured as the master drives a trigger wire to the others) or per-module solves. The carrier has trigger in and out on its connector; Stage 6 decides whether to use it.
- **Identity:** each module's board ID sets its NT topics (`hardware_id`) and its CAN device number.
- **Shared CAN-FD bus:** one 64-byte frame takes about 150 µs at a 5 Mbps data rate. Four modules at 120 FPS, one frame each, is about 7% of a dedicated vision bus, or about 14% if most frames need two parts. Keep the vision modules off the motor buses.

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
| Corner undistortion on RPU | tens of µs |
| CAN-FD corner frame on the wire | ≈0.1–0.35 ms depending on data rate, near-constant on a dedicated bus |
| **Total exposure start → corners at the SystemCore** | **~2–6 ms + remaining readout, with low jitter** (stretch: ~1.5–4 ms + remaining readout) |
| PnP in the robot loop | tens of µs per tag; doesn't change the measurement's capture time |

### Readout is the floor

A global-shutter sensor exposes every row at the same moment, but still reads the rows out one at a time over MIPI. A tag near the bottom of the image waits for almost a full readout. Once the PL pipeline exists, exposure and readout dominate the budget, and the remaining levers are on the sensor side:

- **Sensor windowing:** read out only the rows where tags can appear. 1280×400 instead of 1280×800 roughly halves the worst-case readout.
- **More MIPI lanes or a faster lane rate.**
- **Short exposure,** with brighter tags or IR illumination (check the lighting rules).

This is also why the FPGA build uses MIPI rather than USB cameras (see [Sensors](#sensors)).

Object detection and TagSLAM aren't on this path. They have their own, looser budgets in their stage docs.

### What this buys on the robot

The SystemCore robot loop runs every 10 ms, and pose estimators already compensate for latency using the measurement timestamp. So:

- The **biggest wins are jitter and timestamp accuracy.** A measurement whose age is known to ±0.1 ms fuses better than one known to ±5 ms, especially when the robot is turning fast.
- Lower absolute latency helps auto-align settle faster and reduces how far back the estimator has to replay odometry.
- Robot code still reads once per loop (see the [one-loop robot contract](#one-loop-robot-contract)); the exposure timestamps make that wait harmless for accuracy.
- At 120 FPS against a 100 Hz loop, a module delivers one camera frame most loops and two in some. Robot code must read **all** of them and solve each with its own capture time. Otherwise some of the extra frame rate is thrown away.

Stage 1 builds an **LED glass-to-robot rig** (an LED the robot toggles, seen by the camera, timestamped on both sides) so both systems are measured the same way.

## Bandwidth and memory budget

| Stream | Rate |
|---|---|
| One camera, 1280×800, 8-bit (mono, or raw Bayer for color), 120 FPS | ≈123 MB/s |
| Downscaled stream for object detection, e.g. 320×320 RGB at 30–60 FPS | ≈9–18 MB/s |
| VCU encoding one stream (frame reads plus reference frames) | roughly 0.25–0.5 GB/s (estimate) |
| Encoded video to the SSD, match mode | ≈0.75–1.5 MB/s (6–12 Mbps) |
| DDR frame store writes + DPU weights/activations + Linux | a few GB/s at most |

Zynq UltraScale+ modules use a wide DDR4 interface shared by PL, RPU and APU. Bandwidth is not expected to be the limit, but the DPU and the VCU compete with the frame store, so Stages 3 and 7 measure memory contention and check that the tag path is unaffected. On-chip RAM (block RAM / UltraRAM) for line buffers, tile statistics, label tables and the DPU is the tighter budget; Stage 0b estimates it per block.

**On-chip strip buffer for decode read-back.** Refinement and decode only read pixels of tags that have just closed. So the PL can keep a rolling buffer of the last N rows in UltraRAM (for example 256 rows × 1280 px ≈ 320 KB) and read from it with fixed 1–2-cycle latency, instead of going to DDR. Tags taller than N rows fall back to the DDR frame store. This also keeps the tag path out of DDR contention with the DPU. A full on-chip double buffer (≈2 MB for two 1280×800 frames) would use most of the K26's UltraRAM, which the DPU also wants; Stage 0b checks the numbers.

## Hardware

### Selection criteria (Stage 0b)

1. SoC FPGA with Linux-capable application cores, real-time cores and FPGA logic sharing DDR.
2. MIPI CSI-2 receive for one camera.
3. Enough logic, DSP and on-chip RAM for one tag pipeline, a useful DPU, the downscaler and CAN-FD, with ≥30% headroom (see [Per-module resource estimate](#per-module-resource-estimate)).
4. Vitis AI (or equivalent) supports a DPU on the part.
5. An off-the-shelf module and dev kit available now, so we can prototype before designing our own board.
6. Covered by the free tier of the vendor toolchain.
7. Cost within FRC per-item limits, and the module as a whole near the [cost target](#module-cost).
8. A hardware video encoder (an "EV" Zynq UltraScale+ part; EG parts have none) and PCIe for an M.2 SSD.
9. Small enough for a Limelight-sized module, with heat the housing can remove.

### Candidates

| Option | Why | Concerns |
|---|---|---|
| **AMD Kria K26 SOM + dev kit (KV260 or KR260)** (chosen) | Zynq UltraScale+ EV MPSoC with A53 + R5F + PL + VCU + DDR4 on one module; dev kits with camera connectors and Ethernet; vendor Ubuntu and PetaLinux images; Vitis AI DPU examples. Room for the tag pipeline and a mid-size DPU on one chip | Confirm current Vitis AI support level, free-tier coverage and price. The module's footprint (≈77 × 60 mm) sets the minimum module size |
| Smaller Zynq UltraScale+ SOMs (ZU2/ZU3/ZU4 class) | Same chip family and toolchain; smaller, cooler, sometimes cheaper | Much less room for a DPU next to the tag pipeline; must be an EV part to keep the video encoder. Considered only if a K26 is unavailable |
| Zynq-7020 + external object-detection accelerator (e.g. Hailo-8) | Cheaper FPGA for tag-only modules | Rejected: no VCU, classic Zynq is too small for the tag pipeline plus recording, and the add-on box costs about what the K26 saves |
| Flash-based FPGA (e.g. Microchip PolarFire SoC, Lattice CrossLink-NX / Certus-NX) | Instant-on: configuration survives power loss | Much less logic and RAM, no DPU, different toolchain. Not pursued |
| Newer SoC FPGAs (e.g. AMD Versal AI Edge Gen 2) | Much faster application cores and AI engines | Availability in small modules, cost, toolchain maturity and power. Watch; re-check in Stage 0b |

**Why the K26 for one SKU:** with stereo and VIO gone, one K26 runs a single tag pipeline, object detection on the DPU and recording on the VCU, with no second board. The cheaper-FPGA options save less than an add-on accelerator would cost, and lose the video encoder.

All part-specific numbers (logic, DSP, RAM, MIPI ports, price) get filled in and verified in Stage 0b.

### Per-module resource estimate

Rough estimates, to be replaced by synthesis results in Stage 0b. The K26 has roughly 117k LUTs and 1,250 DSPs (check), so ≥30% headroom means using at most about 82k LUTs.

| Block | LUTs | DSPs | On-chip RAM |
|---|---|---|---|
| MIPI RX, unpacking, timestamps, trigger (plus debayer to luma for color sensors) | ~3–6k | 0–10 | small |
| Tag detection, 4B with the 4C tracer | ~25–40k | ~20–50 | ~2–3 Mbit (incl. strip buffer) |
| Downscaler for the DPU | ~2–4k | ~5–20 | small |
| DPU (size is the knob) | ~20–50k | ~150–450 | ~2–6 Mbit |
| CAN-FD controller | ~2–4k | 0 | small |
| Interconnect, DMA, frame store, VCU interface | ~10–15k | 0 | small |
| **Module total** | **~62–119k** | **~175–530** | **~5–10 Mbit** |

- **The DPU size is set to fit the headroom rule:** Stage 0b picks the largest DPU configuration that keeps the total at or under ~82k LUTs, after the tag pipeline's real synthesis numbers are in. A smaller DPU means a smaller model or lower object FPS, never a slower tag path.
- **A53 load per module (estimate):** NT and recording ~0.5 core, object-detection control and localization ~0.5 core, NAPA cross-check ~0.1 core. Plenty left for the NAPA fallback (~0.5–1 core at 120 FPS) when it's switched on.

### Sensors

The same carrier takes either camera board, chosen per module (pin compatibility checked in Stage 0b):

| Sensor | Resolution and rate | Use |
|---|---|---|
| **OV9281** (mono, baseline) | 1280×800 at 120 FPS | Best tag performance per dollar |
| **OV9782** (color sibling of the OV9281) | 1280×800 at 120 FPS | Better object detection; tags run on the debayered luma, with slightly lower corner accuracy (measured in Stage 4) |
| **AR0234** (mono or color, upgrade) | 1920×1200 at 120 FPS | More range and field of view; more pixel throughput, more DPU input scaling and higher cost |

- **Camera boards:** our own small boards (sensor, regulators, oscillator, connector) for the cheapest build, or off-the-shelf boards for prototypes.
- **Lenses:** standard M12 lenses. Stage 0b tests a few focal lengths and keeps a tested list: narrower for tag range, wider for object detection near the robot.
- **Why MIPI, not USB:** a USB (UVC) camera hands complete frames to Linux, and many have a bridge chip that buffers internally, so the PL can't start until the frame is done. MIPI CSI-2 sends uncompressed RAW8/RAW10 rows straight into the PL, one packet per row, with frame start/end markers. Leave the sensor's optional DPCM compression off.
- **Cable length:** MIPI flex cables are fragile and limited to roughly tens of centimeters. Inside a module the camera board sits a few centimeters from the carrier on a short, enclosed cable, so this mostly matters for dev-kit prototypes.

### Module form factor

About the size of a Limelight or smaller: roughly 3.5 × 2.5 × 1.5 in (estimate, set in Stage 9).

- **Stack, front to back:** camera board centered on the front plate → short MIPI flex → carrier board (M.2 2230 SSD underneath) → K26 SOM → housing back plate as heatsink.
- **Back panel:** power, Ethernet, CAN-FD, and trigger in/out on one locking connector.
- **No IMU required.** Heading comes from the robot's gyro. The carrier can leave an unpopulated footprint for one.
- **Heat:** estimated draw is K26 under load ~6–10 W, camera ~0.3 W, SSD writing ~1–2 W, so ~8–12 W. The aluminum housing is the heatsink, the SoC is thermally bonded to it, and a small filtered fan is added only if Stage 9 measures it's needed. Chip temperature goes on the status topic.
- **Housing:** machined in-house on a 3-axis mill from aluminum plate: a pocketed shell and a back plate, with mounting holes matching common camera mounts.
- **Power:** 12 V through a buck-boost regulator, or PoE over the Ethernet cable for a single cable (check the FRC power rules).

### Module cost

Estimates for a small batch, checked against real quotes in Stage 0b:

| Part | Estimate |
|---|---|
| K26 SOM (commercial grade) | ~$250–330 |
| Camera board + M12 lens | ~$25–60 |
| Carrier board (PCB + assembly, small batch) | ~$60–100 |
| M.2 2230 NVMe SSD | ~$25–40 |
| Housing (aluminum stock, machined in-house) and hardware | ~$15–25 |
| Connectors, cables, CAN-FD transceiver, regulator | ~$25 |
| **Per module** | **~$400–580** |

Dev kits, spares and the first board revision's setup costs are on top. For comparison, COTS cameras with object detection cost about $600 each.

## Toolchain

| Purpose | Tool |
|---|---|
| RTL | SystemVerilog |
| Simulation | Verilator, cocotb (Python testbenches) |
| FPGA build and block design | AMD Vivado (free tier) |
| RPU firmware | Vitis / ARM bare-metal GCC, FreeRTOS or bare metal, OpenAMP/RPMsg |
| Linux image | PetaLinux/Yocto, or the vendor Ubuntu image for Kria, with PREEMPT_RT |
| APU apps | C++20, CMake, WPILib `ntcore`/`wpimath` (same as [`src/`](../src/)), OpenCV |
| Robot library | Java (WPILib), with an optional C++ core over JNI |
| Object detection | Vitis AI (training and quantization in its Docker container) |
| TagSLAM | Desktop C++ with a factor-graph library |
| Golden models | C++ host build sharing code with [`src/`](../src/), the AprilTag C library, NAPA, OpenCV ArUco and ArUco Nano |
| CI | GitHub Actions: lint (Verible), Verilator + cocotb sims, golden-model, APU and robot-library unit tests. Bitstream builds run nightly or on a self-hosted runner, since Vivado is large |
| Board and enclosure | KiCad, FreeCAD (CAM for the 3-axis mill) |

## Block design

### Capture

- MIPI CSI-2 RX from vendor IP, unpacked to 8-bit pixels on an AXI-Stream with start-of-frame and end-of-line markers. Color sensors get a small PL debayer that produces a luma stream for the tag pipeline and RGB for the downscaler.
- The PL drives the sensor's trigger input, from its own timer or from the module trigger input. A 64-bit free-running counter latches the exposure strobe; this is the measurement timestamp.
- Sensor setup (exposure, gain, resolution, window) over I2C from the APU, driven by `/cameras/<hardware_id>/config`. The RPU applies the cached config at boot, before Linux is up.
- `hardware_id` comes from the board ID, which fixes the "no serial number" problem we had with USB ThriftyCams.

### FrameBus

- An AXI-Stream switch that fans the camera stream out to its consumers (tag pipeline, downscaler for the DPU, DDR writer).
- Every frame is also written to a DDR ring buffer that PL, RPU and APU can all read. Consumers never wait for this copy; it's for decode and refinement read-back (below), object detection, setup-mode streaming and recording.
- Consumers that fall behind drop frames and count drops; they never back-pressure the sensor.

### Detectors and their roles

The FPGA detector is the only detector on the live tag path. Four software detectors support it.

| Detector | Runs on | Role |
|---|---|---|
| **FPGA streaming detector** ([below](#apriltag-detector)) | PL, plus RPU for repairs | The competition tag path: a tag finishes ~0.1 ms after the scan passes it, with fixed timing and no CPU use |
| **NAPA** ([zwatx/NAPA](https://github.com/zwatx/NAPA), GPLv3) | APU | First working tag path on the module (Stage 3), live fallback, and a 1-in-N background cross-check in matches. Also the source of the scorecard-style quad finding, ID lookup, refinement and shape repairs |
| **AprilTag 3 library** | Desktop (APU if needed) | Reference: golden model for threshold and blob labeling, and the parity baseline, since today's CPU pipeline uses it |
| **OpenCV ArUco** | APU in setup mode, and desktop | Camera calibration with ChArUco boards; multi-scale threshold comparison in Stage 1 |
| **ArUco Nano** | Desktop | Golden model for the contour tracer (4C); speed and accuracy comparison |

- **NAPA on the A53s.** Its published benchmarks (by the author and forum users, not ours) report ~0.5 ms per 1080p frame on one i7 core and ~0.7–1.6 ms on a Raspberry Pi 5–class board. A rough guess for the A53s is 2–6 ms per frame; Stage 3 measures it. NAPA waits for full frames, so it's roughly 5–15 ms slower than the streaming detector. That's why it's the fallback, not the primary.
- **Fallback switch:** a config setting moves a module's tag path from the FPGA detector to NAPA without reflashing. NAPA's corners go out on the same CAN-FD frames, so robot code doesn't change.
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

### Corner output (RPU)

- Undistort each corner with the module's camera calibration. The robot already sends `camera_matrix` and `distortion_coefficients` in the config topics, and the module caches them in flash for boot. The output is undistorted pixel coordinates, so robot code only needs the pinhole intrinsics it already has.
- Pack the camera frame's tags into one or more [CAN-FD corner frames](#interfaces-and-message-formats) and send them right away, then hand the same detections to module-side PnP.
- Runs on the RPU, so it works before Linux has booted.

### Robot-side PnP

PnP runs in robot code on the SystemCore, in a small GompeiVision robot library.

- **Per camera frame:** IPPE-square for each tag (two solutions), then a multi-tag refinement (Gauss-Newton on reprojection error) when more than one tag is visible. Inputs: the corners, the module's intrinsics and mounting position, and the tag map, all of which robot code already holds.
- **Heading-constrained solve:** the gyro heading at capture time (from the pose estimator's history, `sampleAt()`) constrains or picks between the IPPE solutions. See [Heading and pose disambiguation](#heading-and-pose-disambiguation).
- **Joint solve across modules (optional):** with a shared trigger, one solve uses every module's corners from the same exposure, which helps most when each module only sees one or two tags. Without it, each module's frames are solved separately and the pose estimator fuses them. Stage 6 compares the two.
- **Cost:** tens of µs per tag and roughly 0.1–1 ms per loop for four modules on the SystemCore's Cortex-A76 cores (measured in Stage 5).
- **Java first.** Preallocated arrays and no per-frame object creation, so the garbage collector stays out of the hot path. If profiling shows it's too slow, the solver moves to C++ behind one JNI call per loop that takes all of the loop's frames and writes results into a preallocated `double[]` or direct `ByteBuffer`. JNI itself adds well under a microsecond per call; garbage-collection pauses are the real risk.
- **Why on the robot:** the robot owns the gyro, the tag map, camera mounting positions and the pose estimator, and it sees every module. Moving PnP there removes the heading round trip, lets students change the solver in normal robot code, and means tag-map or mounting changes never need pushing to modules. Latency is unchanged: the solve time moves, and the capture timestamp doesn't.

### Module-side PnP (RPU, NT only)

- IPPE-square (single tag, two solutions) and SQPnP (multi-tag) ported from OpenCV to the RPU in C++, plus tag angles as in [`TagAngleCalculator.cpp`](../src/cpp/estimator/TagAngleCalculator.cpp).
- The R5F has a double-precision FPU, so the math can match the golden model closely.
- Published over NT as `observations`, in the CPU pipeline's format, for dashboards, debugging, roboRIO robots and teams not using the robot library. It isn't fused by robot code that uses the CAN-FD corners (see the [one-loop robot contract](#one-loop-robot-contract)).

### Heading and pose disambiguation

Single-tag PnP has two solutions. Using the robot's heading to pick between them, or to constrain the solve, is the idea behind MegaTag2-style localization. With PnP on the robot, all of this uses the robot's own gyro, with no round trip:

1. **Pick between the IPPE solutions** using the gyro heading at the capture time, looked up in the pose estimator's history.
2. **Heading-constrained solve:** fix yaw to the gyro heading at capture time and solve for position (and pitch/roll if needed). Most robust for single far tags.
3. **Multi-tag solve without a heading constraint** when several tags are visible and quality gates pass (≥2 tags, low reprojection error, tags close and large, slow rotation). Its heading can be used to check or correct the gyro's field offset; never feed a heading-constrained solve's output back into the gyro offset, since that is circular and lets errors reinforce.

### Object recognition and localization (DPU + APU)

- A small single-shot detector, trained on our game-piece datasets and quantized to INT8 with Vitis AI. It runs on every module, on color or mono sensors (color helps when game pieces are told apart by color).
- The DPU is AMD's prebuilt accelerator IP in the PL; the APU loads the model and runs inference through the Vitis AI runtime.
- A PL downscaler on the FrameBus prepares the DPU input, so the APU never resizes full frames.
- [APU] ground-plane projection of each box's bottom-center using camera extrinsics, published as ObjectData with the exposure timestamp. With one camera, range is accurate only for pieces resting on the floor; stacked, airborne or partly hidden pieces get a flag and a larger uncertainty.

### TagSLAM (offline field calibration)

- A desktop tool, not a live block. After a practice session or before matches on a new field, it reads logged corners from every module (from the robot's WPILOG and the modules' recording metadata) plus robot odometry, and solves a factor graph over robot poses and tag landmarks with field-layout priors.
- Output: a refined tag map for that field, loaded into robot code (which does the PnP) and sent to the modules over `/field/tag_<id>` for module-side PnP.
- Works with one camera per module. Several modules seeing the same tags make the map better.
- Could later run on the APU at normal priority if live refinement turns out to be useful; it never touches the tag path.

### IO and networking

- **CAN-FD** is the tag path ([below](#can-fd-tag-path)).
- WPILib `ntcore` over the chip's Gigabit Ethernet for everything else: module-side poses, objects, status, config, field layout and recording control. NT4 details (time sync, reconnects, announce/subscribe) come from the maintained library.
- Config and field ingest reuse [`ConfigInterface.cpp`](../src/cpp/io/ConfigInterface.cpp) and [`FieldInterface.cpp`](../src/cpp/io/FieldInterface.cpp).
- A PREEMPT_RT publisher thread for module-side poses; other publishers at normal priority.
- Hardware exposure timestamps are converted to NT server time using `ntcore`'s time-sync offset and sent as the value timestamp, as the CPU version does.
- HTTP server for the setup-mode stream, a status page and firmware updates.

### CAN-FD tag path

SystemCore has CAN-FD buses; the roboRIO's CAN is classic CAN 2.0 (8-byte frames at 1 Mbps), so roboRIO robots use the NT path with module-side PnP instead.

- **Dedicated bus:** the modules and SystemCore alone on one of SystemCore's buses, terminated with 120 Ω at both ends. No motor traffic to wait behind, and a faulty module can't disturb drivetrain control.
- **Controller:** the Zynq's built-in CAN controllers are classic CAN only. Use a CAN-FD controller in PL (AMD's CAN FD IP, check its license, or an open-source core) driven by the RPU, or an external MCP2518FD over SPI (adds tens of µs). The transceiver must be CAN-FD rated (TJA1462, TCAN1042 or MCP2562FD class).
- **Timing:** a 64-byte frame takes about 0.35 ms at a 2 Mbps data rate, 0.16 ms at 5 Mbps and 0.11 ms at 8 Mbps, with near-constant transit on a dedicated bus.
- **IDs:** FRC extended IDs with manufacturer "Team Use" and device type "Miscellaneous", read on the robot through WPILib's `CAN` class. Check how the SystemCore API exposes FD frames and bus selection.
- **No clock sync needed:** each frame carries the measurement's *age* at send time, and robot code computes `capture time = receive timestamp − age − fixed transit`.
- **Robot side:** drained once per loop under the [one-loop robot contract](#one-loop-robot-contract). WPILib's custom-device `CAN` class keeps only the newest packet per message ID, so the module puts the frame counter and part index into the API index (for example, frame counter mod 4 × 2 parts = 8 slots) and robot code reads all slots each loop, keeping the new ones. Check whether SystemCore's API offers a real receive queue instead.
- Because the RPU drives it, this path works before Linux has finished booting.

### Video recording (VCU + APU)

- The VCU (the Zynq's hard H.264/H.265 encoder) reads the camera stream from the DDR frame store. Its total capacity is about 4K at 60 FPS (≈500 Mpx/s); one 1280×800 stream at 120 FPS is ≈123 Mpx/s, about a quarter, leaving room for the setup stream. Recording at 30–60 FPS is enough for match replays.
- Mono sensors: encode as monochrome (4:0:0) if the VCU and our tool version support it; otherwise fill the chroma planes with a constant in PL. Color sensors encode as normal 4:2:0.
- [APU] GStreamer with AMD's VCU plugins writes **segmented** files (a few seconds each, fragmented MP4 or MKV) to an ext4 or f2fs data partition on the M.2 SSD, separate from the read-only root. A power cut loses at most the last segment.
- A **metadata file** per recording: frame ID, hardware exposure timestamp, detections, corners and poses for each frame, and a sync point to the robot's AdvantageKit/WPILOG log. TagSLAM and the datasets use it.
- Two modes: **match** (H.265, 30–60 FPS, ≈6–12 Mbps per module, ≈0.1–0.25 GB per match) and **dataset** (raw frames, ≈123 MB/s per module, for golden-model datasets and object-detection training; practice sessions only, since compression changes pixel values).
- Started and stopped from robot code over NT (for example, start at match start).
- Recording never touches the tag path; Stage 3's contention test confirms it.

### Storage (APU)

- Files on eMMC: board ID, cached config and camera calibration, DPU model, the saved tag map. No SD card on the robot (they work loose and corrupt on power cuts); the root filesystem is read-only. Video and dataset recordings go to the M.2 SSD.

## Interfaces and message formats

- **Pixel streams:** AXI-Stream, 8 bits per pixel, with frame start / line end sideband and a frame ID.
- **Control:** AXI-Lite registers per PL block, generated from one register description file and exposed to Linux (UIO or a small driver) and the RPU.
- **APU ↔ RPU:** OpenAMP/RPMsg for control, shared-memory rings for data.
- **FrameBus messages:** a versioned, fixed-layout binary format for everything above the pixel level: tag detections, corners, poses, object boxes, map updates. Each message carries the frame ID and exposure timestamp. The same format is used in every shared-memory ring, so blocks can move between PL, RPU and APU.
- **CAN-FD corner frame (draft):** one 64-byte frame carries up to 3 tags. A camera frame with more tags sends more parts back to back. **Every part is self-contained** (its own frame counter and age), so robot code can solve whatever parts arrived this loop and never waits for a missing one.

  | Bytes | Field |
  |---|---|
  | 0–1 | Frame counter (diagnostics, CAN slot rotation, and matching parts of one camera frame within a loop) |
  | 2–3 | Age at send time, µs (up to 65 ms) |
  | 4 | Flags: format version, part index, last part, shared-trigger exposure, NAPA fallback active |
  | 5 | Tags in this part (0–3) |
  | 6 | Tags in the whole camera frame |
  | 7 | Reserved |
  | 8–25 | Tag 0: ID (uint8), decision margin (uint8), 4 corners TL/TR/BR/BL as x, y (int16 each, 1/32 px, undistorted, relative to the principal point, ±1024 px) |
  | 26–43 | Tag 1, same layout |
  | 44–61 | Tag 2, same layout |
  | 62–63 | Reserved |

  A frame with zero tags is still sent at a low rate as a heartbeat, so robot code can tell "no tags" from "module offline".

## Robot interface

### NetworkTables contract

The module keeps the CPU pipeline's topics, so existing robot code and dashboards keep working:

| Direction | Topic | Contents |
|---|---|---|
| Robot → module | `/cameras/<hardware_id>/config/*` | `role`, `camera_matrix`, `distortion_coefficients`, `exposure`, `gain`, `width`, `height`, `compressed`, `setup_mode` |
| Robot → module | `/field/tag_<id>` | 7 doubles per tag (x, y, z, qw, qx, qy, qz) |
| Module → robot | `/cameras/<hardware_id>/output/observations` | Module-side poses: packed double array, same layout as [`OutputPublisher.cpp`](../src/cpp/io/OutputPublisher.cpp), with the exposure timestamp |
| Module → robot | `/cameras/<hardware_id>/output/connected`, `capture_fps`, `processing_fps` | Status |

New outputs (object data, detector status, cross-check mismatches, chip temperature, recording status) get new topics under `/cameras/<hardware_id>/output/`, defined in the stage that adds them. Recording start/stop is a new robot → module topic, defined in Stage 3.

The NT contract works with both the roboRIO and SystemCore. On SystemCore, the CAN-FD corner frames are the measurement path, and `observations` is for display and debugging.

### One-loop robot contract

Everything the robot controller receives from the modules is fully handled within the loop it's read in. The only vision state that lives longer is the pose estimator's own history (odometry and gyro samples, and the measurements it has absorbed).

- **Each message is self-contained.** One CAN-FD part carries capture timing, tag IDs, quality and corners. Parts of one camera frame are solved together when they arrive in the same loop; a part that arrives next loop is solved on its own then. Nothing is held waiting for another part.
- **Absolute time, computed on arrival.** CAN-FD frames carry an age that robot code turns into an absolute capture time using the receive time. NT values carry the exposure time in NT server time. No clock-sync state in robot code.
- **Drain, solve, use, discard.** Each loop:
  1. Read every new CAN-FD part from every module.
  2. Compute capture times and sort by capture time.
  3. Solve PnP for each camera frame (or one joint solve per shared-trigger exposure), using the gyro heading at the capture time from the pose estimator's history.
  4. Reject bad solutions against the current estimate and quality gates (decision margin, reprojection error, tag area, pose jump), set standard deviations, and call `addVisionMeasurement(pose, captureTime, stdDevs)`.
  5. Drop the corners. In Java, overwrite preallocated arrays next loop rather than allocating.
- **No multi-frame filtering in robot code.** Checks like "seen in 3 frames in a row" would be state that outlives the loop. Use per-frame quality gates instead. State that has to persist (drop and duplicate counting, consistency across frames, camera health, timeouts) lives on the module and comes back as status.
- **One path per measurement.** Robot code fuses either the CAN-FD corners (SystemCore, with the robot library) or the NT `observations` (roboRIO, or without the library), never both, so nothing needs deduplication across loops.
- **Budget:** 4 modules × 120 FPS is about 5 camera frames per 10 ms loop. Each `addVisionMeasurement()` replays odometry from its capture time, and each frame needs a PnP solve, so Stage 5 measures the per-loop cost at the worst-case tag count.

## Verification strategy

Each layer must pass before the next is trusted:

1. **Golden models** (`model/`): C++ models of each PL and RPU block, built from the CPU code, NAPA and ArUco Nano (see [Detectors and their roles](#detectors-and-their-roles)). For integer blocks the model defines the exact expected output. The robot library's solver is tested against the same models.
2. **Unit simulation:** cocotb + Verilator testbenches per PL block, compared with the golden model on synthetic and recorded frames. Integer blocks must match bit for bit; floating-point blocks must stay within a stated tolerance.
3. **Frame-replay simulation:** recorded match and practice-field videos (captured with the CPU system) replayed through the full simulated pipeline, comparing detections and poses with the CPU output.
4. **Hardware-in-the-loop:** recorded frames loaded into the DDR frame store from Linux and fed into the real PL pipeline in place of the sensor, so on-chip results can be compared frame by frame.
5. **Latency rig:** the LED glass-to-robot test from Stage 1, run on both systems.
6. **Load tests:** tag-path latency measured while the DPU and recording run flat out, to prove the tiers don't interfere.
7. **On-robot A/B:** CPU and FPGA units side by side on the practice robot, logging both to AdvantageKit/WPILOG.
8. **Power-cut recovery:** cut power mid-stream and measure how long until the tag path produces CAN-FD corners again, and until NT is back.
9. **Thermal:** a closed module on a hot robot, with every feature and recording running for a full practice day, chip temperature logged.

## Risk register

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| Not enough HDL experience on the team | High | High | Custom hardware limited to the AprilTag detector; training path (below); mentor review; keep the CPU system in competition |
| Effort spans several seasons and students graduate | High | High | Stage docs with exit criteria, golden models and tests as the "spec", pair new members with block owners |
| Quad fit / boundary clustering doesn't fit timing or RAM in PL | Medium | Medium | Run quad fit on the RPU first; PL fit engines later; or 4B with the 4C tracer |
| SystemCore's CAN-FD API or bus setup doesn't suit a custom device | Medium | Medium | Check in Stage 0b; NT path with module-side PnP still works; raw UDP from the RPU as an alternative fast path |
| Robot-side solve too slow, or garbage-collection pauses in Java | Low | Medium | Preallocated arrays, measured in Stage 5; C++ solver behind one JNI call per loop |
| DPU too small to fit next to the tag pipeline with headroom | Medium | Medium | DPU size is the knob; smaller model or lower object FPS; tag pipeline has priority |
| DPU and the frame store contend for DDR | Medium | Medium | Load tests in Stage 7; QoS settings on the memory interconnect; smaller DPU |
| Vitis AI support for Zynq UltraScale+ changes or lapses | Medium | Medium | Check in Stage 0b; pin versions; CPU-only inference on the A53s at lower FPS as a fallback |
| Single-camera object range is poor for pieces off the floor | High | Low | Flag and inflate uncertainty; several modules seeing the same piece |
| No localization between tag sightings under heavy defense | Medium | Medium | Several modules facing different directions; slip detection on wheel odometry; fast reacquisition from the streaming detector |
| Color sensor (OV9782) costs tag accuracy | Medium | Low | Mono OV9281 on modules that mainly do tags; measured in Stage 4 |
| MIPI signal integrity on our own board | Medium | High | Stay on the dev kit through Stage 8; carrier board for the SOM, not a full chip-down design; vendor layout guides |
| FRC rule changes (power, cost, custom electronics) | Low | High | Check the game manual every season |
| Electrical noise and brownouts on the robot | Medium | High | Buck-boost regulator that holds the input steady through ~7 V sags, brownout-tolerant boot, watchdog, read-only root filesystem, eMMC boot (no SD card), status topic |
| Module reboots mid-match (hit or brownout) and is blind while Linux boots | Medium | High | Tag path and CAN-FD boot from the first-stage loader without Linux; recovery time measured in Stage 9 |
| MIPI flex cable cracks or works loose on the robot | Medium | High | Locked mounting and strain relief, short enclosed run, spares |
| Corner scorecard (4B) misses tags that touch dark background or are occluded | Medium | Low | Measured in Stage 1; on-demand contour tracer (4C); or use 4A |
| FPGA detector bug or bad bitstream at an event | Low | High | NAPA fallback by config switch; 1-in-N cross-check catches silent errors |
| Gyro offset corrected by its own constrained-solve output (feedback loop) | Low | Medium | Only unconstrained, quality-gated multi-tag headings may correct the offset |
| Module overheats in a Limelight-sized housing (~8–12 W) | Low | High | Housing as heatsink, optional fan, temperature on the status topic, thermal test in Stage 9; lower DPU rate or recording FPS when hot |
| Costs creep above the target | Medium | Medium | Quotes in Stage 0b; cost table kept current; cheapest camera board and in-house housing |
| SSD corruption on power cuts | Medium | Low | Segmented recording, separate data partition, journaling filesystem |

## FRC rules checklist

Check each item against the current season's game manual; rule numbers change every year.

- [ ] Custom circuits and coprocessors are allowed and connect to the control system only through allowed interfaces (Ethernet through the robot radio/switch, and CAN).
- [ ] Custom CAN devices are allowed, use the "Team Use" manufacturer and "Miscellaneous" device type IDs, and bus wiring and termination follow the rules.
- [ ] Frame grounding and ESD requirements are met for the module and its enclosure.
- [ ] Powered from the PDH/PDP on an appropriately rated breaker or fuse (if using PoE, the injector and wiring are allowed).
- [ ] Each module's cost (SOM, carrier board, camera, SSD, housing) within the per-item cost limit, and every module counted in the cost accounting.
- [ ] No wireless radios active on the board (disable any Wi-Fi/Bluetooth on dev kits).
- [ ] Any illumination LEDs follow the lighting rules.
- [ ] Mounting and enclosure meet the robot construction rules.

## Team and skills

- **Three tracks.** A small hardware track (SystemVerilog, simulation, Vivado) owns capture and the AprilTag detector. A software track (Linux C++, real-time firmware, ML) owns the module's other blocks. Robot programmers own the robot library and PnP, in the Java they already use.
- **Training path for the hardware track:** digital logic basics → HDL exercises (e.g. HDLBits) → Verilator + cocotb on a small block → owning a block from golden model to hardware.
- **Ownership:** each block has a student owner and a reviewer. Every RTL change needs a passing simulation and a review.
- **Keep it teachable:** short design notes per block in `docs/`, readable testbenches, and no "only one person understands this" blocks.

## Notes on the structure diagram

- [`GompeiVisionStructureDiagram.dot`](../GompeiVisionStructureDiagram.dot) shows one single-lens module and the robot code. Tag corners go to the robot over CAN-FD, where PnP and the pose estimator run. Stereo, VIO and the onboard IMU are gone, and TagSLAM is an offline tool fed by logs.
- The diagram doesn't show where each node runs. The README's architecture section adds the PL / RPU / APU mapping.
