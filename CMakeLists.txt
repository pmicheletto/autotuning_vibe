cmake_minimum_required(VERSION 3.15)
project(Autotuning LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(AUTOTUNING_BUILD_TESTS "Build the unit/integration tests" ON)
# Test framework: installed GoogleTest if present; otherwise the bundled
# minigtest runner (test/support/minigtest, same macros and CTest protocol),
# or the real GoogleTest downloaded with -DAUTOTUNING_FETCH_GTEST=ON.
option(AUTOTUNING_FETCH_GTEST "Download GoogleTest when it is not installed (needs network)" OFF)

enable_testing()

# ------------------------------------------------------------------------------
# FMI4cpp (vendored copy, locally patched). Builds offline: its libzip/pugixml
# dependencies are replaced by bundled code, see lib/FMI4cpp/third_party/.
# ------------------------------------------------------------------------------
add_subdirectory(lib/FMI4cpp)

# ------------------------------------------------------------------------------
# autotuning: statistics + tuning algorithms + FMI co-simulation plant.
# ------------------------------------------------------------------------------
add_library(autotuning STATIC
  # statistics
  statistics/JerkEstimator.cpp
  statistics/WelfordAlgorithm.cpp
  # tuning
  tuning/control_report.cpp
  tuning/control_validation.cpp
  tuning/loop_control.cpp
  tuning/oscillation_counter.cpp
  tuning/settling_detector.cpp
  tuning/steady_state_detector.cpp
  tuning/tuning_task.cpp
  tuning/parameters/parameter_validator.cpp
  tuning/parameters/skogestad_tuner.cpp
  tuning/transfer_function/sundaresan.cpp
  # FMI simulation (the only ISystem implementation)
  tuning/system/system_simulation.cpp
  simulation/ModelicaWrapper.cpp
  simulation/live_plot.cpp
)
target_include_directories(autotuning PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(autotuning PUBLIC fmi4cpp::fmi4cpp)

# ------------------------------------------------------------------------------
# Executables
# ------------------------------------------------------------------------------
# `sim` locates FMUs relative to its own location (build/sim -> ../simulation/
# modelica_models), so it runs from any working directory; the source tree is
# compiled in as a fallback for out-of-tree build directories.
add_executable(sim main.cpp)
target_link_libraries(sim PRIVATE autotuning)
target_compile_definitions(sim PRIVATE
  AUTOTUNING_MODELS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/simulation/modelica_models"
  AUTOTUNING_TOOLS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tools")

# The gtest suites hard-code ../simulation/modelica_models/ and therefore
# expect to be run from the build directory (ctest does that).

# ------------------------------------------------------------------------------
# Tests
# ------------------------------------------------------------------------------
if(AUTOTUNING_BUILD_TESTS)
  find_package(GTest QUIET)
  if(GTest_FOUND)
    message(STATUS "Tests: using installed GoogleTest")
  elseif(AUTOTUNING_FETCH_GTEST)
    message(STATUS "Tests: downloading GoogleTest (needs network)")
    include(FetchContent)
    FetchContent_Declare(
      googletest
      URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
      DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    FetchContent_MakeAvailable(googletest)
  else()
    message(STATUS "Tests: GoogleTest not installed, using the bundled minigtest runner "
                   "(-DAUTOTUNING_FETCH_GTEST=ON to download GoogleTest)")
    add_library(minigtest_main STATIC test/support/minigtest/minigtest_main.cpp)
    target_include_directories(minigtest_main PUBLIC test/support/minigtest)
    add_library(GTest::gmock_main ALIAS minigtest_main)
  endif()

  include(GoogleTest)

  function(autotuning_add_test TARGET_NAME)
    add_executable(${TARGET_NAME} ${ARGN})
    target_link_libraries(${TARGET_NAME} PRIVATE autotuning GTest::gmock_main)
    gtest_discover_tests(${TARGET_NAME} WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
  endfunction()

  # Core tuning logic and its math; plant-side tests run against the
  # deterministic FOPDT model in test/support/fake_plant.h (no FMU needed).
  autotuning_add_test(sim_unit_tests
    test/unit/ControlValidationTest.cpp
    test/unit/JerkEstimatorTest.cpp
    test/unit/ModelicaWraperTest.cpp
    test/unit/OscillationCounterTest.cpp
    test/unit/ParameterValidatorTest.cpp
    test/unit/SettlingDetectorTest.cpp
    test/unit/SkogestadTunerTest.cpp
    test/unit/SteadyStateDetectorTest.cpp
    test/unit/SundaresanIdentificationTest.cpp
    test/unit/TuningTaskTest.cpp
    test/unit/WelfordAlgorithmTest.cpp
  )

  autotuning_add_test(sim_integration_tests
    test/integration/JerkFmuClosedLoopTest.cpp
    test/integration/SimulationTest.cpp
  )
endif()
