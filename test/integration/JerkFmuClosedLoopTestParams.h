#ifndef TEST_INTEGRATION_JERKFMUCLOSEDLOOPTESTPARAMS_H_
#define TEST_INTEGRATION_JERKFMUCLOSEDLOOPTESTPARAMS_H_

#include <cstddef>

// Integration test: jerk pipeline + same CSV columns as SimulationVarsTxt / main.
struct JerkFmuClosedLoopTestParams {
  static constexpr const char* kFmuRel = "../simulation/modelica_models/variantA.fmu";
  static constexpr const char* kOutFileName = "jerk_fmu_closed_loop_trace.txt";

  static constexpr double kInnerFmuStep = 0.001;

  static constexpr int kUdpIntervalSteps = 100;
  static constexpr double kJerkCadenceDt =
      kInnerFmuStep * static_cast<double>(kUdpIntervalSteps);

  static constexpr int kUdpSampleCount = 125;

  static constexpr double TotalSimSeconds() {
    return static_cast<double>(kUdpSampleCount) * kJerkCadenceDt;
  }

  // PID gains copied from a run that produced sim_trace.txt for comparable traces.
  static constexpr double kPidKp = 0.00449063;
  static constexpr double kPidTi = 3.89197;
  static constexpr double kPidTd = 0.0199368;

  static constexpr unsigned kJerkSpeedMeanWindow = 20;
  static constexpr unsigned kJerkMeanWindow = 20;
  static constexpr double kJerkMinSampleIntervalSec = 0.05;

  static constexpr double kStepScheduleTimes[] = {0.0};
  static constexpr double kStepScheduleHeights[] = {36.0517};
  static constexpr std::size_t kStepScheduleCount =
      sizeof(kStepScheduleTimes) / sizeof(kStepScheduleTimes[0]);

  static constexpr double StepHeightAt(double t) {
    double h = kStepScheduleHeights[0];
    for (std::size_t i = 0; i < kStepScheduleCount; ++i) {
      if (kStepScheduleTimes[i] <= t) {
        h = kStepScheduleHeights[i];
      }
    }
    return h;
  }
};

#endif  // TEST_INTEGRATION_JERKFMUCLOSEDLOOPTESTPARAMS_H_
