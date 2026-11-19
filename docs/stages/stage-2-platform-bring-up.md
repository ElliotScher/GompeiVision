# Stage 2: Platform bring-up

## Goal

The dev kit boots our Linux image, runs the existing CPU pipeline, talks to the real-time cores, and shares memory between FPGA logic, RPU and APU.

## Deliverables

- `platform/`: Vivado block design, device tree and a reproducible Linux image build (PetaLinux/Yocto, or the vendor Ubuntu image with our packages), with PREEMPT_RT.
- **The legacy CPU pipeline from [`src/`](../../src/) running on the APU** with a USB camera, publishing to NetworkTables. This proves the Linux side, `ntcore` and the build, and gives a first latency number for the new board.
- `rpu/`: firmware for the R5F cores, loaded from Linux with OpenAMP, exchanging messages over RPMsg.
- Shared-memory rings between PL, RPU and APU using the FrameBus message format, with a test that a PL block writes, the RPU processes and the APU reads.
- `apu/`: the NT publisher process with a real-time thread for tag measurements, plus config and field ingest reused from [`ConfigInterface.cpp`](../../src/cpp/io/ConfigInterface.cpp) and [`FieldInterface.cpp`](../../src/cpp/io/FieldInterface.cpp).
- Hardware timestamp counter in PL readable from APU and RPU, converted to NT server time.
- Boot flow where the first-stage boot loader loads the bitstream, the RPU firmware and the cached config from flash, and Linux attaches to the already-running RPU (remoteproc attach) instead of starting it.

## Exit criteria

- [ ] The legacy pipeline runs on the APU and robot code reads its output unchanged.
- [ ] A dummy measurement goes PL → RPU → APU → NT and robot code reads it, with the hardware timestamp converted correctly (offset error measured; target under 0.5 ms).
- [ ] Publisher-thread jitter measured under load (all four A53s busy) and recorded.
- [ ] Reconnects correctly after the network drops or the robot reboots.
- [ ] 24-hour soak with no crashes.
- [ ] The RPU is running and producing a dummy measurement within a few seconds of power-on, before Linux has booted.

## Risks

- Linux image builds are slow and fiddly. Start from the vendor's image and change as little as possible.
- If remoteproc attach doesn't work with our kernel and OpenAMP versions, fall back to Linux loading the RPU, and record the boot-time cost here.
