#include "WelfordAlgorithm.h"

#include <algorithm>
#include <cmath>

WelfordAlgorithm::WelfordAlgorithm(unsigned int window_size)
    : window_(std::max(window_size, 1u), 0.0) {}

void WelfordAlgorithm::AddSample(double new_value) {
  if (count_ < window_.size()) {
    count_++;
  } else {
    const double oldest = window_[window_index_];
    sum_ -= oldest;
    sum_squares_ -= oldest * oldest;
  }
  window_[window_index_] = new_value;
  window_index_ = (window_index_ + 1) % window_.size();

  sum_ += new_value;
  sum_squares_ += new_value * new_value;
  mean_ = sum_ / count_;
  m2_ = std::max(0.0, sum_squares_ - sum_ * sum_ / count_);
}

double WelfordAlgorithm::GetVariance() const {
  if (count_ < window_.size() || count_ < 2) {
    return 0.0;
  }
  return m2_ / (count_ - 1);
}

double WelfordAlgorithm::GetDeviation() const { return std::sqrt(GetVariance()); }

void WelfordAlgorithm::Clear() {
  std::fill(window_.begin(), window_.end(), 0.0);
  count_ = 0;
  window_index_ = 0;
  mean_ = 0.0;
  m2_ = 0.0;
  sum_ = 0.0;
  sum_squares_ = 0.0;
}
