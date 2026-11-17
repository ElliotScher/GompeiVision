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

## Exit criteria

- [ ] The legacy pipeline runs on the APU and robot code reads its output unchanged.
- [ ] A dummy measurement goes PL → RPU → APU → NT and robot code reads it, with the hardware timestamp converted correctly (offset error measured; target under 0.5 ms).
- [ ] Publisher-thread jitter measured under load (all four A53s busy) and recorded.
- [ ] Reconnects correctly after the network drops or the robot reboots.
- [ ] 24-hour soak with no crashes.

## Risks

- Linux image builds are slow and fiddly. Start from the vendor's image and change as little as possible.
