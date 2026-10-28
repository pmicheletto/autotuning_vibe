#include <gtest/gtest.h>

#include <cmath>

#include "statistics/JerkEstimator.h"

TEST(JerkEstimatorTest, DecimationSkipsDenseCalls) {
  JerkEstimator jerk(3, 0.1);
  EXPECT_FALSE(jerk.AddSpeed(1.0, 0.0).has_value());
  EXPECT_FALSE(jerk.AddSpeed(2.0, 0.05).has_value());
  EXPECT_FALSE(jerk.AddSpeed(3.0, 0.1).has_value());
  EXPECT_FALSE(jerk.AddSpeed(4.0, 0.15).has_value());
  EXPECT_TRUE(jerk.AddSpeed(5.0, 0.2).has_value());
}

TEST(JerkEstimatorTest, ConstantVelocityJerkNearZero) {
  JerkEstimator jerk(1, 0.05);
  for (int i = 0; i < 20; ++i) {
    const double t = i * 0.05;
    jerk.AddSpeed(10.0, t);
  }
  const auto j = jerk.GetLastJerk();
  ASSERT_TRUE(j.has_value());
  EXPECT_NEAR(*j, 0.0, 1e-6);
}

TEST(JerkEstimatorTest, ConstantAccelerationJerkNearZero) {
  JerkEstimator jerk(1, 0.05);
  for (int i = 0; i < 30; ++i) {
    const double t = i * 0.05;
    jerk.AddSpeed(2.0 * t, t);
  }
  const auto j = jerk.GetLastJerk();
  ASSERT_TRUE(j.has_value());
  EXPECT_NEAR(*j, 0.0, 1e-3);
}

TEST(JerkEstimatorTest, MaxAbsJerkTracksPeak) {
  JerkEstimator jerk(1, 0.05);
  double t = 0.0;
  for (int i = 0; i < 10; ++i) {
    jerk.AddSpeed(0.0, t);
    t += 0.05;
  }
  for (int i = 0; i < 10; ++i) {
    jerk.AddSpeed(10.0, t);
    t += 0.05;
  }
  EXPECT_GT(jerk.GetMaxAbsJerk(), 0.0);
}
