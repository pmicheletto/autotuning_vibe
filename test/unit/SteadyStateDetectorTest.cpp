#include <gtest/gtest.h>

#include <cmath>

#include "tuning/steady_state_detector.h"

namespace {

SteadyStateDetector::Config config(double windowS = 5.0, double relTol = 0.01, double absTol = 1e-3) {
  SteadyStateDetector::Config c;
  c.dt = 0.001;
  c.windowS = windowS;
  c.holdS = windowS;
  c.relTol = relTol;
  c.absTol = absTol;
  return c;
}

}  // namespace

TEST(SteadyStateDetector, WindowStatisticsMatchDirectComputation) {
  // 3-sample windows: after 6 samples, current = {4,5,6}, previous = {1,2,3}.
  SteadyStateDetector::Config c = config();
  c.windowS = 0.003;
  c.holdS = 0.001;
  SteadyStateDetector d(c);
  for (double v : {1.0, 2.0, 3.0, 4.0, 5.0, 6.0}) d.update(v);
  EXPECT_TRUE(d.windowsFull());
  EXPECT_NEAR(d.mean(), 5.0, 1e-12);
  EXPECT_NEAR(d.previousMean(), 2.0, 1e-12);
  EXPECT_NEAR(d.deviation(), 1.0, 1e-12);  // sample std-dev of {4,5,6}
  d.update(7.0);                            // current = {5,6,7}, previous = {2,3,4}
  EXPECT_NEAR(d.mean(), 6.0, 1e-12);
  EXPECT_NEAR(d.previousMean(), 3.0, 1e-12);
}

TEST(SteadyStateDetector, ConstantSignalIsSteadyAfterTwoWindowsPlusHold) {
  SteadyStateDetector d(config());
  int steadyAt = -1;
  for (int i = 1; i <= 20000 && steadyAt < 0; ++i) {
    if (d.update(23.7)) steadyAt = i;
  }
  // The condition is first evaluable when both windows are full (sample
  // 10000) and must then hold for 5000 samples: sample 14999.
  EXPECT_EQ(steadyAt, 2 * 5000 + 5000 - 1);
  EXPECT_NEAR(d.mean(), 23.7, 1e-9);
  EXPECT_NEAR(d.deviation(), 0.0, 1e-9);  // two-pass: no Σx²/n - m² cancellation
}

TEST(SteadyStateDetector, ExponentialApproachIsDeclaredSteadyNearItsFinalValue) {
  // y = 23.7 (1 - exp(-t/4.25)); with W = 5 s and 1% tolerance the means of
  // consecutive windows agree once the remaining error is ~1.5%, so the
  // reported level is within 2% of the true final value and the detection
  // happens long after the rise (> 4 tau) but without waiting forever.
  const double yInf = 23.7, tau = 4.25, dt = 0.001;
  SteadyStateDetector d(config());
  double steadyAt = -1.0;
  for (int i = 1; i <= 200000 && steadyAt < 0.0; ++i) {
    if (d.update(yInf * (1.0 - std::exp(-i * dt / tau)))) steadyAt = i * dt;
  }
  ASSERT_GT(steadyAt, 0.0);
  EXPECT_GT(steadyAt, 4.0 * tau);
  EXPECT_LT(steadyAt, 12.0 * tau);
  EXPECT_NEAR(d.mean(), yInf, 0.02 * yInf);
}

TEST(SteadyStateDetector, SlowRampIsNeverSteady) {
  // A 0.1 units/s ramp around level 20: consecutive 5 s windows differ by 0.5
  // (2.5%) > 1% tolerance, however small its instantaneous variation is.
  SteadyStateDetector d(config());
  bool steady = false;
  for (int i = 1; i <= 60000; ++i) steady = d.update(20.0 + 0.1 * i * 0.001);
  EXPECT_FALSE(steady);
}

TEST(SteadyStateDetector, PeriodicWobbleAroundALevelIsSteady) {
  // +/-0.5 wobble with 2.5 s period on 23.7 (two periods per window): window
  // means agree, and the std-dev ~0.35 is reported as the noise at that level.
  SteadyStateDetector d(config());
  bool steady = false;
  for (int i = 1; i <= 20000; ++i) steady = d.update(23.7 + 0.5 * std::sin(2 * M_PI * i * 0.001 / 2.5));
  EXPECT_TRUE(steady);
  EXPECT_NEAR(d.mean(), 23.7, 0.01);
  EXPECT_NEAR(d.deviation(), 0.5 / std::sqrt(2.0), 0.02);
}

TEST(SteadyStateDetector, AbsoluteToleranceGovernsNearZero) {
  // Level 0.01 with 1% relative tolerance would demand 1e-4 agreement; the
  // absolute floor (1e-3) applies instead.
  SteadyStateDetector d(config());
  EXPECT_NEAR(SteadyStateDetector(config()).tolerance(), 1e-3, 1e-15);
  bool steady = false;
  for (int i = 1; i <= 20000; ++i) steady = d.update(0.01 + 5e-4 * std::sin(i * 0.01));
  EXPECT_TRUE(steady);
}

TEST(SteadyStateDetector, ResetForgetsHistory) {
  SteadyStateDetector d(config());
  for (int i = 0; i < 20000; ++i) d.update(1.0);
  ASSERT_TRUE(d.steady());
  d.reset();
  EXPECT_FALSE(d.steady());
  EXPECT_FALSE(d.windowsFull());
  EXPECT_EQ(d.samples(), 0u);
}

TEST(SteadyStateDetector, RejectsInvalidConfiguration) {
  auto bad = config();
  bad.windowS = 0.0;
  EXPECT_THROW(SteadyStateDetector{bad}, std::invalid_argument);
}
