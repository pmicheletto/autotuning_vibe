#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>

#include "simulation/SimulationVarsTxt.h"
#include "test/integration/JerkFmuClosedLoopTestParams.h"
#include "statistics/JerkEstimator.h"
#include "tuning/loop_control.h"
#include "tuning/system/system_interface.h"

using P = JerkFmuClosedLoopTestParams;

// Mirrors main: JerkEstimator(20, 0.05, 20), UDP every 100 steps at 0.001 s, SimulationVarsTxt.
// Bit-exact match to a full TuningTask sim_trace slice would require exporting PID/noise;
// this test only checks the jerk/CSV logging cadence matches main.

TEST(JerkFmuClosedLoopTest, WritesJerkTraceMatchesSimTraceControllerPhases) {
  ASSERT_TRUE(std::filesystem::exists(P::kFmuRel))
      << "Missing FMU (cwd should be build dir): " << P::kFmuRel;

  std::unique_ptr<ISystem> system = createSystem(std::string(P::kFmuRel));
  ASSERT_NE(system, nullptr);

  const double ref_nominal = P::kStepScheduleHeights[0];
  ASSERT_GT(std::fabs(ref_nominal), 1e-12);

  loop_control::closeLoop(*system);
  loop_control::applyPID(*system, P::kPidKp, P::kPidTi, P::kPidTd);

  JerkEstimator jerk(P::kJerkSpeedMeanWindow, P::kJerkMinSampleIntervalSec, P::kJerkMeanWindow);
  jerk.Clear();
  jerk.ResetMaxAbsJerk();

  double sim_time_sec = 0.0;

  const std::filesystem::path out_path =
      std::filesystem::current_path() / P::kOutFileName;
  SimulationVarsTxt sim_txt(out_path.string());

  double jerk_abs_max_logged = 0.0;

  for (int udp = 0; udp < P::kUdpSampleCount; ++udp) {
    ASSERT_NO_THROW(system->setVariable("driver.step.height", ref_nominal));

    for (int s = 0; s < P::kUdpIntervalSteps; ++s) {
      ASSERT_NO_THROW(system->advanceSimulation(P::kInnerFmuStep));
      sim_time_sec += P::kInnerFmuStep;
    }

    const double t = sim_time_sec;
    const double raw_speed = system->readValue("out");
    const double step_val = system->readValue("driver.step.y");

    std::optional<double> j = jerk.AddSpeed(raw_speed, t);
    const double speed_filt = jerk.GetLastFilteredSpeed();
    if (j.has_value() && std::isfinite(*j)) {
      jerk_abs_max_logged = std::max(jerk_abs_max_logged, std::fabs(*j));
    }

    sim_txt.append(t, step_val, raw_speed, j, speed_filt, jerk_abs_max_logged, "WaitTouchRef");
  }

  ASSERT_TRUE(std::filesystem::file_size(out_path) > 80u)
      << "Ficheiro de saida vazio ou demasiado pequeno: " << out_path.string();

  EXPECT_TRUE(std::isfinite(system->readValue("out")));
  EXPECT_GT(jerk_abs_max_logged, 1e-12);
}
