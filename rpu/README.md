# rpu/

Firmware for the Cortex-R5F real-time cores (FreeRTOS or bare metal), loaded from Linux with OpenAMP and talking to the APU over RPMsg: corner undistortion and CAN-FD output, TagAngleCalculator, and module-side PnP (IPPE-square and SQPnP).

- **Toolchain:** `arm-none-eabi-gcc` (see [`tools/versions.env`](../tools/versions.env)).
- **Filled in by:** Stages [2](../docs/stages/stage-2-platform-bring-up.md) and [5](../docs/stages/stage-5-pose-and-mvp.md).
- **CI:** host-built unit tests of the portable math (undistortion, PnP) will run against `model/`.
