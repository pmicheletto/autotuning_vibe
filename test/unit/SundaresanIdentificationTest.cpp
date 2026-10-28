#include <gtest/gtest.h>

#include <cmath>

#include "test/support/fake_plant.h"
#include "tuning/loop_control.h"
#include "tuning/transfer_function/sundaresan.h"

namespace {

constexpr double kDt = 0.001;

// Runs the identification state machine on `plant` until it finishes.
// Returns the simulated time used.
double runIdentification(FakePlant& plant, Sundaresan& sundaresan, double maxSimS = 600.0) {
  loop_control::openLoop(plant);
  for (int i = 0; i < static_cast<int>(maxSimS / kDt); ++i) {
    plant.advanceSimulation(kDt);
    sundaresan.Execute();
    if (sundaresan.IsFinished()) break;
  }
  return plant.time();
}

}  // namespace

TEST(SundaresanIdentification, RecoversFopdtParametersOfAKnownPlant) {
  // Sundaresan-Krishnaswamy two-point method: t1 at 35.3%, t2 at 85.3% of the
  // final value; tau = 0.67 (t2 - t1), theta = 1.3 t1 - 0.29 t2, K = y_inf / step.
  const double K = 40.0, tau = 4.25, theta = 0.5;
  FakePlant plant(K, tau, theta);
  Sundaresan sundaresan(plant);

  runIdentification(plant, sundaresan);
  ASSERT_TRUE(sundaresan.IsFinished());
  ASSERT_EQ(sundaresan.GetState(), SundaresanState::Done);

  const PlantModel& m = sundaresan.GetPlantModel();
  // The steady state is declared slightly early by the std-dev < noise rule,
  // which biases the final value (and thus K and the time points) by ~1%.
  EXPECT_NEAR(m.K, K, 0.02 * K);
  EXPECT_NEAR(m.tau, tau, 0.06 * tau);
  EXPECT_NEAR(m.theta, theta, 0.15);
  EXPECT_GT(sundaresan.GetNoise(), 0.0);
  EXPECT_NEAR(sundaresan.GetFinalValue(), K * 0.5, 0.02 * K * 0.5);  // second step is 0.5
}

TEST(SundaresanIdentification, DeadTimeFreePlantGivesNearZeroTheta) {
  FakePlant plant(40.0, 3.0, 0.0);
  Sundaresan sundaresan(plant);
  runIdentification(plant, sundaresan);
  ASSERT_EQ(sundaresan.GetState(), SundaresanState::Done);
  EXPECT_NEAR(sundaresan.GetPlantModel().tau, 3.0, 0.06 * 3.0);
  EXPECT_LT(sundaresan.GetPlantModel().theta, 0.1);
  EXPECT_GE(sundaresan.GetPlantModel().theta, 0.0);  // clamped, never negative
}

TEST(SundaresanIdentification, IncreasesTheProbeStepUntilThePlantMoves) {
  // K = 5: a 0.01 step gives 0.05 < movement threshold 0.2, so the probe step
  // must be raised (by 1.0 every 5 s) before movement is detected.
  FakePlant plant(5.0, 2.0, 0.0);
  Sundaresan sundaresan(plant);
  runIdentification(plant, sundaresan);
  ASSERT_EQ(sundaresan.GetState(), SundaresanState::Done);
  EXPECT_NEAR(sundaresan.GetPlantModel().K, 5.0, 0.1);
}

TEST(SundaresanIdentification, UnresponsivePlantIsReportedNotHung) {
  FakePlant plant(0.0, 2.0, 0.0);  // no gain at all
  Sundaresan sundaresan(plant);
  runIdentification(plant, sundaresan, 600.0);
  EXPECT_TRUE(sundaresan.IsFinished());
  EXPECT_EQ(sundaresan.GetState(), SundaresanState::UnresponsivePlant);
}
