cmake_minimum_required(VERSION 3.14)
project(vessel_host_tests CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)

# GoogleTest (pinned).
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG v1.15.2
  GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(googletest)

# The real Arduino MIDI Library (pinned). Header-only for our usage; we just
# need its src/ on the include path so <MIDI.h> resolves to the genuine parser.
# Populate only (do NOT add_subdirectory): the library's own CMakeLists bundles
# a second google-test that collides with ours.
FetchContent_Declare(
  arduino_midi_library
  GIT_REPOSITORY https://github.com/FortySevenEffects/arduino_midi_library.git
  GIT_TAG 5.0.2
  GIT_SHALLOW TRUE)
FetchContent_GetProperties(arduino_midi_library)
if(NOT arduino_midi_library_POPULATED)
  FetchContent_Populate(arduino_midi_library)
endif()

set(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)

add_executable(vessel_test vessel_test.cpp host_sim.cpp)

# ARCH_HOST selects the host simulation branches in the firmware headers.
target_compile_definitions(vessel_test PRIVATE ARCH_HOST)

target_include_directories(
  vessel_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}                       # host mocks: Arduino.h, USB-MIDI.h
  ${REPO_ROOT}                                      # firmware headers + vessel.ino
  ${arduino_midi_library_SOURCE_DIR}/src)           # real <MIDI.h>

target_link_libraries(vessel_test PRIVATE gtest_main)

enable_testing()

include(GoogleTest)
gtest_discover_tests(vessel_test)

# Enforce clang-format LLVM style across all C/C++ sources. Fails (non-zero)
# if any tracked source file is not formatted, so CI catches drift.
add_test(
  NAME clang_format
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/tools/check-format.sh
  WORKING_DIRECTORY ${REPO_ROOT})
