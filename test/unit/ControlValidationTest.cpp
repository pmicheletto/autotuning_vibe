#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "tuning/control_validation.h"

namespace {

constexpr double kDt = 0.001;
constexpr double kRef = 10.0;
constexpr double kBand = 0.02 * kRef;
constexpr double kHoldS = 5.0;
constexpr double kSsWindowS = 1.0;

std::vector<double> firstOrder(double tau, double durationS) {
  std::vector<double> y;
  for (int i = 1; i <= static_cast<int>(durationS / kDt); ++i) {
    y.push_back(kRef * (1.0 - std::exp(-i * kDt / tau)));
  }
  return y;
}

std::vector<double> secondOrder(double zeta, double wn, double durationS) {
  const double wd = wn * std::sqrt(1.0 - zeta * zeta);
  std::vector<double> y;
  for (int i = 1; i <= static_cast<int>(durationS / kDt); ++i) {
    const double t = i * kDt;
    y.push_back(kRef * (1.0 - std::exp(-zeta * wn * t) / std::sqrt(1.0 - zeta * zeta) *
                                  std::sin(wd * t + std::acos(zeta))));
  }
  return y;
}

}  // namespace

TEST(ControlValidation, FirstOrderResponseMatchesAnalyticValues) {
  const double tau = 2.0;
  const auto m = computeStepResponseMetrics(firstOrder(tau, 30.0), kDt, kRef, kBand, kHoldS, kSsWindowS);

  EXPECT_NEAR(m.riseTimeS, tau * std::log(9.0), 0.002);     // t90 - t10
  EXPECT_NEAR(m.overshootPercent, 0.0, 1e-9);
  EXPECT_TRUE(m.settled);
  EXPECT_NEAR(m.settlingTimeS, tau * std::log(50.0), 0.002);  // enters the 2% band
  EXPECT_EQ(m.oscillations, 0);
  EXPECT_NEAR(m.steadyStateError, 0.0, 1e-3);
  EXPECT_NEAR(m.iae, kRef * tau, 0.01);                      // integral of r*exp(-t/tau)
}

TEST(ControlValidation, UnderdampedSecondOrderOvershootAndPeak) {
  const double zeta = 0.5, wn = 1.0;
  const auto m = computeStepResponseMetrics(secondOrder(zeta, wn, 30.0), kDt, kRef, kBand, kHoldS, kSsWindowS);

  const double expectedOvershoot = 100.0 * std::exp(-M_PI * zeta / std::sqrt(1.0 - zeta * zeta));
  EXPECT_NEAR(m.overshootPercent, expectedOvershoot, 0.01);
  EXPECT_NEAR(m.peakTimeS, M_PI / (wn * std::sqrt(1.0 - zeta * zeta)), 0.002);
  EXPECT_TRUE(m.settled);
  EXPECT_NEAR(m.settlingTimeS, 8.08, 0.05);
  // One undershoot lobe (2.66% deep) follows the first overshoot; the next peak is inside the band.
  EXPECT_EQ(m.oscillations, 1);
}

TEST(ControlValidation, RampThatNeverReachesTheBandIsNotSettled) {
  std::vector<double> y;
  for (int i = 1; i <= 10000; ++i) y.push_back(0.5 * kRef * i * kDt / 10.0);
  const auto m = computeStepResponseMetrics(y, kDt, kRef, kBand, kHoldS, kSsWindowS);

  EXPECT_FALSE(m.settled);
  EXPECT_TRUE(std::isnan(m.settlingTimeS));
  EXPECT_TRUE(std::isnan(m.riseTimeS));  // 90% never reached
  EXPECT_EQ(m.oscillations, 0);
}

TEST(ControlValidation, SustainedOscillationCountsAlternatingLobes) {
  // 5% amplitude, 2 s period, 5 periods: 10 lobes -> 9 alternations after the first.
  std::vector<double> y;
  for (int i = 1; i <= 10000; ++i) y.push_back(kRef * (1.0 + 0.05 * std::sin(2.0 * M_PI * i * kDt / 2.0)));
  const auto m = computeStepResponseMetrics(y, kDt, kRef, kBand, kHoldS, kSsWindowS);

  EXPECT_EQ(m.oscillations, 9);
  EXPECT_FALSE(m.settled);
  EXPECT_NEAR(m.overshootPercent, 5.0, 0.01);
}

TEST(ControlValidation, EmptyTraceYieldsNoMetrics) {
  const auto m = computeStepResponseMetrics({}, kDt, kRef, kBand, kHoldS, kSsWindowS);
  EXPECT_EQ(m.steps, 0u);
  EXPECT_FALSE(m.settled);
  EXPECT_TRUE(std::isnan(m.riseTimeS));
  EXPECT_TRUE(std::isnan(m.iae));
}
