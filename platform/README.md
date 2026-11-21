# platform/

The Vivado block design (PS configuration, MIPI CSI-2 RX, DPU, VCU and our PL blocks), constraints, the device tree, and the reproducible Linux image build (PetaLinux/Yocto, or the vendor Ubuntu image with our packages) with PREEMPT_RT.

- **Tools:** Vivado, Vitis and PetaLinux at `AMD_TOOLS_VERSION` in [`tools/versions.env`](../tools/versions.env). Bitstreams build on the shared build machine, not in hosted CI.
- **Filled in by:** the Stage [0b](../docs/stages/stage-0b-hardware-and-scaffolding.md) blinky bitstream, then [Stage 2](../docs/stages/stage-2-platform-bring-up.md).
