# Toolchain file for the Cortex-R5F (double-precision VFPv3-D16):
#   cmake -S rpu -B build/rpu-r5f --toolchain cmake/cortex-r5f.cmake
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_CXX_FLAGS_INIT
    "-mcpu=cortex-r5 -mfpu=vfpv3-d16 -mfloat-abi=hard -fno-exceptions -fno-rtti -fno-math-errno")
