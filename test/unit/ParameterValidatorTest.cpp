#include <gtest/gtest.h>

#include <limits>

#include "tuning/parameters/parameter_validator.h"

namespace {
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
}  // namespace

TEST(ParameterValidator, AcceptsWellFormedGains) {
  EXPECT_TRUE(ParameterValidator::validate(PIDParameters{0.5, 4.0, 0.1}));
  EXPECT_TRUE(ParameterValidator::validate(PIDParameters{0.0, 1.0, 0.0}));  // Kp = 0, Td = 0 allowed
}

TEST(ParameterValidator, RejectsNegativeKpOrTd) {
  EXPECT_FALSE(ParameterValidator::validate(PIDParameters{-0.1, 4.0, 0.1}));
  EXPECT_FALSE(ParameterValidator::validate(PIDParameters{0.5, 4.0, -0.1}));
}

TEST(ParameterValidator, RejectsNonPositiveTi) {
  EXPECT_FALSE(ParameterValidator::validate(PIDParameters{0.5, 0.0, 0.1}));
  EXPECT_FALSE(ParameterValidator::validate(PIDParameters{0.5, -1.0, 0.1}));
}

TEST(ParameterValidator, RejectsNonFiniteValues) {
  EXPECT_FALSE(ParameterValidator::validate(PIDParameters{kNaN, 4.0, 0.1}));
  EXPECT_FALSE(ParameterValidator::validate(PIDParameters{0.5, kInf, 0.1}));
  EXPECT_FALSE(ParameterValidator::validate(PIDParameters{0.5, 4.0, kNaN}));
}
