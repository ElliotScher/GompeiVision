# The RPU's portable code, built for the host (tests against the golden model)
# or for the Cortex-R5F:
#
#   cmake -S rpu -B build/rpu && cmake --build build/rpu -j
#   ctest --test-dir build/rpu --output-on-failure
#   cmake -S rpu -B build/rpu-r5f --toolchain rpu/cmake/cortex-r5f.cmake
cmake_minimum_required(VERSION 3.28)
project(gompeivision_rpu LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

add_library(gv_rpu STATIC src/finisher.cpp)
target_include_directories(gv_rpu PUBLIC include)
target_compile_options(gv_rpu PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow)
set_target_properties(gv_rpu PROPERTIES POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_CROSSCOMPILING)
  # For the RTL testbenches.
  add_library(gv_rpu_host SHARED host/capi.cpp)
  target_link_libraries(gv_rpu_host PRIVATE gv_rpu)

  option(GV_RPU_TESTS "Build the tests (needs the golden model and GoogleTest)" ON)
  if(GV_RPU_TESTS)
    enable_testing()
    set(GV_MODEL_TESTS OFF CACHE BOOL "" FORCE)
    add_subdirectory(../model model EXCLUDE_FROM_ALL)
    include(FetchContent)
    FetchContent_Declare(googletest
      GIT_REPOSITORY https://github.com/google/googletest.git
      GIT_TAG v1.17.0
      GIT_SHALLOW TRUE
      EXCLUDE_FROM_ALL)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
    add_executable(gv_rpu_tests tests/test_finisher.cpp ../model/tests/synth.cpp)
    target_include_directories(gv_rpu_tests PRIVATE ../model/tests)
    target_link_libraries(gv_rpu_tests PRIVATE gv_rpu gv_model GTest::gtest_main)
    target_compile_options(gv_rpu_tests PRIVATE -Wall -Wextra -Wconversion -Wshadow)
    include(GoogleTest)
    gtest_discover_tests(gv_rpu_tests)
  endif()
endif()
