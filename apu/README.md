# apu/

Linux C++ apps on the Cortex-A53 cores: the NetworkTables publisher (with a real-time thread for tag measurements), config and field-layout ingest, object detection through Vitis AI, recording to the SSD through the VCU, storage management and the setup stream.

- **Reuses:** [`ConfigInterface.cpp`](../src/cpp/io/ConfigInterface.cpp) and [`FieldInterface.cpp`](../src/cpp/io/FieldInterface.cpp) from the CPU pipeline.
- **Toolchain:** `aarch64-linux-gnu-g++`, CMake and Ninja, with WPILib `ntcore`/`wpimath` at `WPILIB_MODULE_VERSION`.
- **Filled in by:** Stages [2](../docs/stages/stage-2-platform-bring-up.md), [3](../docs/stages/stage-3-camera-capture.md) and [7](../docs/stages/stage-7-object-detection.md).
- **CI:** once `apu/CMakeLists.txt` exists, CI builds natively for the host and runs `ctest`.
