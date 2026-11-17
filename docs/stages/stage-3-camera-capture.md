# Stage 3: Camera capture, timestamps, FrameBus, setup stream

## Goal

Frames from a MIPI global-shutter sensor stream through the FrameBus with hardware exposure timestamps, are visible from Linux, and can be viewed in setup mode.

## Deliverables

- [PL] MIPI CSI-2 RX and pixel unpacking to AXI-Stream with frame ID and sideband.
- [APU] sensor driver over I2C: exposure, gain, resolution, trigger mode, from `/cameras/<hardware_id>/config`.
- [PL] trigger generator and exposure-strobe timestamp latch.
- [PL] FrameBus switch with per-consumer FIFOs, drop counters, and a DDR ring-buffer writer.
- [APU] access to the frame store (UIO or a small driver), and a tool to dump frames.
- [APU] setup mode: MJPEG stream from the frame store over HTTP (or the hardware video encoder if the part has one), enabled by `setup_mode`.
- `hardware_id` from physical port + board ID.

## Exit criteria

- [ ] Sustained target resolution and FPS with zero dropped frames for 1 hour.
- [ ] Exposure timestamp accuracy measured with the LED rig or a scope (target: under 50 µs).
- [ ] Setup-mode stream viewable in a browser and in the robot dashboard.
- [ ] Changing exposure and gain from robot code takes effect within one frame period.
- [ ] Recorded frames can be loaded into the frame store from Linux and replayed into the PL pipeline (for hardware-in-the-loop tests).

## Risks

- MIPI link bring-up can take weeks. Start with the vendor's camera reference design for the dev kit.
