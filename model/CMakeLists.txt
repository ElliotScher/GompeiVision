# Golden models for the FPGA blocks (Stage 1). Standalone host build:
#
#   cmake -S model -B build/model && cmake --build build/model -j
#   ctest --test-dir build/model --output-on-failure
#
# The AprilTag library is fetched at the same tag the CPU pipeline uses
# (Dockerfile.X86), so the models are compared against exactly the code that
# runs today.
cmake_minimum_required(VERSION 3.28)
project(gompeivision_model LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(GV_MODEL_TESTS "Build the golden-model tests" ON)

include(FetchContent)

# Keep in sync with APRILTAG_VERSION in Dockerfile.X86 / Dockerfile.ARM.
set(GV_APRILTAG_TAG v3.4.5)
# Static, so the tests can call its internal threshold() and
# connected_components() directly. This also turns off the Python wrapper.
set(BUILD_SHARED_LIBS OFF)
FetchContent_Declare(apriltag
  GIT_REPOSITORY https://github.com/AprilRobotics/apriltag.git
  GIT_TAG ${GV_APRILTAG_TAG}
  GIT_SHALLOW TRUE
  SYSTEM  # its headers aren't held to our warning flags
  EXCLUDE_FROM_ALL)
FetchContent_MakeAvailable(apriltag)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  set(GV_WARNINGS -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
endif()

add_library(gv_model
  src/image.cpp
  src/decimate.cpp
  src/threshold.cpp
  src/components.cpp
  src/tag_lookup.cpp
  src/quad.cpp
  src/refine.cpp)
target_include_directories(gv_model PUBLIC include)
# tag_lookup reads the tag36h11 code table and cell layout from AprilTag.
target_link_libraries(gv_model PUBLIC apriltag)
target_compile_options(gv_model PRIVATE ${GV_WARNINGS})

add_executable(gv-model tools/gv_model.cpp)
target_link_libraries(gv-model PRIVATE gv_model)
target_compile_options(gv-model PRIVATE ${GV_WARNINGS})

if(GV_MODEL_TESTS)
  enable_testing()
  FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.17.0
    GIT_SHALLOW TRUE
    EXCLUDE_FROM_ALL)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  add_executable(gv_model_tests
    tests/synth.cpp
    tests/test_decimate.cpp
    tests/test_threshold.cpp
    tests/test_components.cpp
    tests/test_synth.cpp
    tests/test_tag_lookup.cpp
    tests/test_quad.cpp
    tests/test_refine.cpp)
  target_link_libraries(gv_model_tests PRIVATE gv_model apriltag GTest::gtest_main)
  target_compile_options(gv_model_tests PRIVATE ${GV_WARNINGS})
  include(GoogleTest)
  gtest_discover_tests(gv_model_tests)
endif()
