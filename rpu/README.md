# rpu/

Code for the Cortex-R5F real-time cores (FreeRTOS or bare metal), loaded from Linux with OpenAMP and talking to the APU over RPMsg: the tag detector's RPU half, corner undistortion and CAN-FD output, TagAngleCalculator, and module-side PnP (IPPE-square and SQPnP).

- **Toolchain:** `arm-none-eabi-gcc` (see [`tools/versions.env`](../tools/versions.env)).
- **Filled in by:** Stages [2](../docs/stages/stage-2-platform-bring-up.md), [4](../docs/stages/stage-4-apriltag-detector.md) and [5](../docs/stages/stage-5-pose-and-mvp.md).
- **CI:** builds the portable code for the host and tests it against [`model/`](../model/).

```sh
cmake -S rpu -B build/rpu && cmake --build build/rpu -j          # host, with tests
ctest --test-dir build/rpu --output-on-failure
cmake -S rpu -B build/rpu-r5f --toolchain cmake/cortex-r5f.cmake  # Cortex-R5F library
cmake --build build/rpu-r5f
```

| Part | Files | Status |
|---|---|---|
| Finisher: rough quads to finished tags | [`finisher.h`](include/gv/rpu/finisher.h), [`finisher.cpp`](src/finisher.cpp) | Same tags as the golden detector on the host and against the simulated PL (`sim/detector`). About 10 KB of code for the R5F; no heap or exceptions. Not yet run on an R5F: registers, interrupts and timing come with the platform |
| C interface for the RTL testbenches | [`host/capi.cpp`](host/capi.cpp) | Host only |
