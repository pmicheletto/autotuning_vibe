#include <gtest/gtest.h>

#include <cmath>

#include "tuning/oscillation_counter.h"

namespace {
constexpr double kBand = 0.72;
constexpr unsigned kMinSteps = 18;
}  // namespace

TEST(OscillationCounter, MinimumLobeDurationComesFromTimeOrSteps) {
  EXPECT_EQ(OscillationCounter::minOutsideSteps(0.001), 80u);   // 80 ms / 1 ms
  EXPECT_EQ(OscillationCounter::minOutsideSteps(0.01), 18u);    // 8 steps < floor of 18
  EXPECT_EQ(OscillationCounter::minOutsideSteps(0.0), 18u);
}

TEST(OscillationCounter, SingleOvershootIsNotAnOscillation) {
  OscillationCounter c(kBand, kMinSteps);
  for (int i = 0; i < 100; ++i) c.update(+2.0);  // one positive lobe
  for (int i = 0; i < 100; ++i) c.update(0.0);
  EXPECT_EQ(c.count(), 0);
}

TEST(OscillationCounter, AlternatingLobesCountOnceEach) {
  OscillationCounter c(kBand, kMinSteps);
  for (int lobe = 0; lobe < 4; ++lobe) {
    const double e = (lobe % 2 == 0) ? +2.0 : -2.0;
    for (int i = 0; i < 100; ++i) c.update(e);
    for (int i = 0; i < 10; ++i) c.update(0.0);
  }
  EXPECT_EQ(c.count(), 3);  // +,-,+,- -> three sign changes
}

TEST(OscillationCounter, ShortExcursionsAreIgnored) {
  OscillationCounter c(kBand, kMinSteps);
  for (int lobe = 0; lobe < 6; ++lobe) {
    const double e = (lobe % 2 == 0) ? +2.0 : -2.0;
    for (int i = 0; i < 10; ++i) c.update(e);  // shorter than kMinSteps
    c.update(0.0);
  }
  EXPECT_EQ(c.count(), 0);
}

TEST(OscillationCounter, HysteresisKeepsALobeAliveInsideTheReleaseBand) {
  // Error drops from +2 to +0.5 (inside the band, above the release level
  // 0.58 * 0.72 = 0.42): still the same lobe, so no new lobe is confirmed.
  OscillationCounter c(kBand, kMinSteps);
  for (int i = 0; i < 100; ++i) c.update(+2.0);
  for (int i = 0; i < 100; ++i) c.update(+0.5);
  for (int i = 0; i < 100; ++i) c.update(-2.0);
  EXPECT_EQ(c.count(), 1);
  c.reset();
  EXPECT_EQ(c.count(), 0);
}
