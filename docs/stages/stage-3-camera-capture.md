# Stage 3: Camera capture, timestamps, FrameBus, setup stream

## Goal

Frames from a MIPI global-shutter sensor stream through the FrameBus with hardware exposure timestamps, are visible from Linux, can be viewed in setup mode, and are recorded to the module's SSD.

## Deliverables

- [PL] MIPI CSI-2 RX and pixel unpacking to AXI-Stream with frame ID and sideband.
- [APU] sensor driver over I2C: exposure, gain, resolution, trigger mode, from `/cameras/<hardware_id>/config`.
- [PL] trigger generator and exposure-strobe timestamp latch.
- [PL] FrameBus switch with per-consumer FIFOs, drop counters, and a DDR ring-buffer writer.
- [APU] access to the frame store (UIO or a small driver), and a tool to dump frames.
- [APU] setup mode: MJPEG stream from the frame store over HTTP (or H.264 from the VCU), enabled by `setup_mode`.
- `hardware_id` from physical port + board ID.
- [APU] video recording (see [PLAN.md](../PLAN.md#video-recording-vcu--apu)): VCU encode (side by side once Stage 6 adds the second lens), segmented files on the M.2 SSD, a per-frame metadata file, start/stop over NT, and match and raw dataset modes.
- Sensor windowing (ROI) and lane-rate settings exposed in config, so readout time can be traded for field of view.

## Exit criteria

- [ ] Sustained target resolution and FPS with zero dropped frames for 1 hour.
- [ ] Exposure timestamp accuracy measured with the LED rig or a scope (target: under 50 µs).
- [ ] Readout time measured at full frame and at a windowed height.
- [ ] Setup-mode stream viewable in a browser and in the robot dashboard.
- [ ] Changing exposure and gain from robot code takes effect within one frame period.
- [ ] Recorded frames can be loaded into the frame store from Linux and replayed into the PL pipeline (for hardware-in-the-loop tests).
- [ ] Recording runs for a full simulated match with no dropped frames, and a power cut mid-recording loses at most the last segment.
- [ ] Recording metadata lines up with a robot WPILOG to the frame.
- [ ] Tag-path timing unchanged while recording.

## Risks

- MIPI link bring-up can take weeks. Start with the vendor's camera reference design for the dev kit.
