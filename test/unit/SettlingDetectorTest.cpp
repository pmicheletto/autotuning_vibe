#include <gtest/gtest.h>

#include <cmath>

#include "tuning/settling_detector.h"

namespace {

SettlingDetector::Config config(double reference = 36.0, double noise = 1e-4, double holdS = 5.0) {
  SettlingDetector::Config c;
  c.dt = 0.001;
  c.reference = reference;
  c.holdS = holdS;
  c.bandFrac = 0.02;
  c.noiseMult = 5.0;
  c.noise = noise;
  return c;
}

}  // namespace

TEST(SettlingDetector, BandIsTwoPercentOfReferenceWithNoiseFloor) {
  EXPECT_NEAR(SettlingDetector::computeBand(config(36.0, 1e-4)), 0.72, 1e-12);
  EXPECT_NEAR(SettlingDetector::computeBand(config(36.0, 1.0)), 5.0, 1e-12);  // 5 * noise dominates
  EXPECT_NEAR(SettlingDetector::computeBand(config(-10.0, 0.0)), 0.2, 1e-12); // |ref|
}

TEST(SettlingDetector, SlowRampIsNotSteadyStateEvenThoughItsStdDevIsSmall) {
  // A 0.05 units/s ramp far from the reference has a tiny windowed std-dev.
  // It must not be reported as settled.
  SettlingDetector d(config());
  bool settled = false;
  for (int i = 1; i <= 20000; ++i) settled = d.update(20.0 + 0.05 * i * 0.001);  // 20 -> 21
  EXPECT_FALSE(settled);
  EXPECT_LT(d.windowDeviation(), d.band());
}

TEST(SettlingDetector, FirstOrderResponseSettlesWhenEnteringBandForGood) {
  // y = r (1 - exp(-t/tau)): enters the 2% band at t = tau ln(50).
  const double tau = 4.0, r = 36.0, dt = 0.001;
  SettlingDetector d(config(r));
  double settledAt = -1.0;
  for (int i = 1; i <= 60000 && settledAt < 0.0; ++i) {
    if (d.update(r * (1.0 - std::exp(-i * dt / tau)))) settledAt = i * dt;
  }
  ASSERT_GT(settledAt, 0.0);
  EXPECT_NEAR(d.settlingTimeS(), tau * std::log(50.0), 0.002);
  EXPECT_NEAR(settledAt, tau * std::log(50.0) + 5.0, 0.002);  // hold window elapsed
}

TEST(SettlingDetector, NoiseLevelWobbleAroundReferenceIsTolerated) {
  // +/- 0.5 wobble around 36 (band 0.72): in band, windowed std ~0.35 < band.
  SettlingDetector d(config());
  bool settled = false;
  for (int i = 1; i <= 10000; ++i) settled = d.update(36.0 + 0.5 * std::sin(2 * M_PI * i * 0.001 / 3.0));
  EXPECT_TRUE(settled);
  EXPECT_NEAR(d.settlingTimeS(), 0.001, 1e-9);  // in band from the first sample
}

TEST(SettlingDetector, LeavingTheBandRestartsTheHoldWindow) {
  SettlingDetector d(config());
  for (int i = 0; i < 4000; ++i) d.update(36.0);     // 4 s in band (hold is 5 s)
  EXPECT_FALSE(d.settled());
  d.update(40.0);                                     // excursion
  for (int i = 0; i < 4999; ++i) d.update(36.0);     // 4.999 s in band
  EXPECT_FALSE(d.settled());
  d.update(36.0);                                     // 5.000 s
  EXPECT_TRUE(d.settled());
  EXPECT_NEAR(d.settlingTimeS(), (4000 + 1 + 1) * 0.001, 1e-9);  // first sample after the excursion
}

TEST(SettlingDetector, ResetClearsState) {
  SettlingDetector d(config());
  for (int i = 0; i < 6000; ++i) d.update(36.0);
  ASSERT_TRUE(d.settled());
  d.reset();
  EXPECT_FALSE(d.settled());
  EXPECT_TRUE(std::isnan(d.settlingTimeS()));
}

TEST(SettlingDetector, RejectsInvalidConfiguration) {
  auto bad = config();
  bad.holdS = 0.0;
  EXPECT_THROW(SettlingDetector{bad}, std::invalid_argument);
}
