#include <gtest/gtest.h>

#include <cmath>

#include "statistics/WelfordAlgorithm.h"

namespace {

void addRange(WelfordAlgorithm& w, int first, int last) {
  for (int v = first; v <= last; ++v) w.AddSample(v);
}

}  // namespace

TEST(WelfordAlgorithmTest, SlidingWindowKeepsOnlyTheLastSamples) {
  WelfordAlgorithm welford(10);
  addRange(welford, 10, 19);            // window = 10..19
  EXPECT_NEAR(welford.GetMean(), 14.5, 1e-12);

  addRange(welford, 20, 24);            // window = 15..24 (oldest five evicted)
  EXPECT_NEAR(welford.GetMean(), 19.5, 1e-12);

  addRange(welford, 25, 29);            // window = 20..29 (full wrap-around)
  EXPECT_NEAR(welford.GetMean(), 24.5, 1e-12);
  EXPECT_NEAR(welford.GetVariance(), 9.1666666667, 1e-9);  // sample variance of 10 consecutive ints
}

TEST(WelfordAlgorithmTest, SlidingWindowWithDifferentWindowSize) {
  WelfordAlgorithm welford(5);
  addRange(welford, 10, 19);            // window = 15..19
  EXPECT_NEAR(welford.GetMean(), 17.0, 1e-12);
  EXPECT_NEAR(welford.GetVariance(), 2.5, 1e-12);
  EXPECT_NEAR(welford.GetDeviation(), std::sqrt(2.5), 1e-12);
}

TEST(WelfordAlgorithmTest, StatisticsOfAFullWindow) {
  WelfordAlgorithm welford(10);
  addRange(welford, 10, 19);
  EXPECT_NEAR(welford.GetMean(), 14.5, 1e-12);
  EXPECT_NEAR(welford.GetVariance(), 9.1666666667, 1e-9);
  EXPECT_NEAR(welford.GetDeviation(), std::sqrt(9.1666666667), 1e-9);
}

TEST(WelfordAlgorithmTest, VarianceIsZeroUntilTheWindowIsFull) {
  WelfordAlgorithm welford(5);
  addRange(welford, 1, 4);              // 4 of 5 samples
  EXPECT_NEAR(welford.GetMean(), 2.5, 1e-12);
  EXPECT_DOUBLE_EQ(welford.GetVariance(), 0.0);
  welford.AddSample(5);
  EXPECT_NEAR(welford.GetVariance(), 2.5, 1e-12);
}

TEST(WelfordAlgorithmTest, ClearForgetsAllSamples) {
  WelfordAlgorithm welford(5);
  addRange(welford, 10, 14);
  welford.Clear();
  EXPECT_DOUBLE_EQ(welford.GetMean(), 0.0);
  EXPECT_DOUBLE_EQ(welford.GetVariance(), 0.0);
  welford.AddSample(3.0);
  EXPECT_NEAR(welford.GetMean(), 3.0, 1e-12);
}
