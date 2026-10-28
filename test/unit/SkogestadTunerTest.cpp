#include <gtest/gtest.h>

#include "tuning/parameters/skogestad_tuner.h"

// SIMC (Skogestad) rules for a FOPDT model K e^{-theta s} / (tau s + 1):
//   Kp = (tau + theta/2) / (K (tau_c + theta/2))
//   Ti = min(tau + theta/2, 4 (tau_c + theta/2))
//   Td = tau theta / (2 tau + theta)        (PID variant; 0 for PI)

TEST(SkogestadTuner, PiGainsFollowSimcFormulas) {
  const PlantModel plant{2.0, 4.0, 0.5};  // K, tau, theta
  const double tc = 3.0;
  const PIDParameters p = SkogestadTuner::calculatePI(plant, tc);

  EXPECT_NEAR(p.Kp, (4.0 + 0.25) / (2.0 * (3.0 + 0.25)), 1e-12);
  EXPECT_NEAR(p.Ti, 4.25, 1e-12);  // tau + theta/2 < 4 (tc + theta/2) = 13
  EXPECT_DOUBLE_EQ(p.Td, 0.0);
}

TEST(SkogestadTuner, PidAddsDerivativeTime) {
  const PlantModel plant{2.0, 4.0, 0.5};
  const PIDParameters p = SkogestadTuner::calculatePID(plant, 3.0);

  EXPECT_NEAR(p.Kp, (4.0 + 0.25) / (2.0 * 3.25), 1e-12);
  EXPECT_NEAR(p.Ti, 4.25, 1e-12);
  EXPECT_NEAR(p.Td, (4.0 * 0.5) / (2.0 * 4.0 + 0.5), 1e-12);
}

TEST(SkogestadTuner, IntegralTimeIsCappedByFourTimesTc) {
  // Integrating-like plant with large tau: Ti switches to 4 (tc + theta/2).
  const PlantModel plant{1.0, 100.0, 1.0};
  const PIDParameters p = SkogestadTuner::calculatePI(plant, 2.0);
  EXPECT_NEAR(p.Ti, 4.0 * (2.0 + 0.5), 1e-12);
  EXPECT_LT(p.Ti, 100.5);
}

TEST(SkogestadTuner, SmallerTcGivesMoreAggressiveGain) {
  const PlantModel plant{47.4, 4.25, 0.0};
  const double kpFast = SkogestadTuner::calculatePI(plant, 6.0).Kp;
  const double kpSlow = SkogestadTuner::calculatePI(plant, 12.0).Kp;
  EXPECT_GT(kpFast, kpSlow);
  EXPECT_NEAR(kpFast / kpSlow, 2.0, 1e-12);  // theta = 0 -> Kp proportional to 1/tc
}

TEST(SkogestadTuner, ZeroGainPlantYieldsZeroParameters) {
  const PlantModel plant{0.0, 4.0, 0.5};
  const PIDParameters p = SkogestadTuner::calculatePID(plant, 3.0);
  EXPECT_DOUBLE_EQ(p.Kp, 0.0);
  EXPECT_DOUBLE_EQ(p.Ti, 0.0);
  EXPECT_DOUBLE_EQ(p.Td, 0.0);
}

TEST(SkogestadTuner, NonPositiveDenominatorYieldsZeroParameters) {
  const PlantModel plant{2.0, 4.0, 0.0};
  const PIDParameters p = SkogestadTuner::calculatePI(plant, 0.0);  // tc + theta/2 == 0
  EXPECT_DOUBLE_EQ(p.Kp, 0.0);
  EXPECT_DOUBLE_EQ(p.Ti, 0.0);
}

TEST(SkogestadTuner, PureDelayPlantStillTunes) {
  const PlantModel plant{1.0, 0.0, 2.0};  // tau = 0
  const PIDParameters p = SkogestadTuner::calculatePID(plant, 1.0);
  EXPECT_NEAR(p.Kp, 1.0 / (1.0 * 2.0), 1e-12);  // (0 + 1) / (1 * (1 + 1))
  EXPECT_NEAR(p.Ti, 1.0, 1e-12);                // min(1, 8)
  EXPECT_DOUBLE_EQ(p.Td, 0.0);                  // tau * theta = 0
}
