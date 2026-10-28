#include "JerkEstimator.h"

#include <algorithm>
#include <cmath>

namespace {

std::optional<double> ComputeJerkFromTriple(double t0, double v0, double t1, double v1,
                                            double t2, double v2) {
  const double dt01 = t1 - t0;
  const double dt12 = t2 - t1;
  if (dt01 <= 0.0 || dt12 <= 0.0) {
    return std::nullopt;
  }
  const double a0 = (v1 - v0) / dt01;
  const double a1 = (v2 - v1) / dt12;
  const double dt_mean = 0.5 * (dt01 + dt12);
  if (dt_mean <= 0.0) {
    return std::nullopt;
  }
  return (a1 - a0) / dt_mean;
}

}  // namespace

JerkEstimator::JerkEstimator(unsigned int speed_mean_window, double min_sample_interval_sec,
                           unsigned int jerk_mean_window)
    : speed_filter_(speed_mean_window),
      jerk_filter_(jerk_mean_window),
      min_sample_interval_sec_(min_sample_interval_sec),
      last_accept_time_(std::nullopt),
      t_hist_{0.0, 0.0, 0.0},
      v_hist_{0.0, 0.0, 0.0},
      vel_points_(0),
      last_jerk_(std::nullopt),
      last_filtered_speed_(0.0),
      max_abs_jerk_(0.0) {}

bool JerkEstimator::ShouldAcceptSample(double time_sec) const {
  if (!last_accept_time_.has_value()) {
    return true;
  }
  return (time_sec - *last_accept_time_) >= min_sample_interval_sec_;
}

std::optional<double> JerkEstimator::AddSpeed(double raw_speed, double time_sec) {
  if (last_accept_time_.has_value() && time_sec < *last_accept_time_) {
    return std::nullopt;
  }
  if (!ShouldAcceptSample(time_sec)) {
    return std::nullopt;
  }

  last_accept_time_ = time_sec;

  speed_filter_.AddSample(raw_speed);
  const double v_f = speed_filter_.GetMean();
  last_filtered_speed_ = v_f;

  if (vel_points_ < 3) {
    t_hist_[vel_points_] = time_sec;
    v_hist_[vel_points_] = v_f;
    vel_points_++;
    if (vel_points_ < 3) {
      return std::nullopt;
    }
  } else {
    t_hist_[0] = t_hist_[1];
    v_hist_[0] = v_hist_[1];
    t_hist_[1] = t_hist_[2];
    v_hist_[1] = v_hist_[2];
    t_hist_[2] = time_sec;
    v_hist_[2] = v_f;
  }

  const auto jerk_raw =
      ComputeJerkFromTriple(t_hist_[0], v_hist_[0], t_hist_[1], v_hist_[1], t_hist_[2], v_hist_[2]);
  if (!jerk_raw.has_value()) {
    return std::nullopt;
  }
  jerk_filter_.AddSample(*jerk_raw);
  const double jerk_f = jerk_filter_.GetMean();
  last_jerk_ = jerk_f;
  max_abs_jerk_ = std::max(max_abs_jerk_, std::abs(jerk_f));
  return jerk_f;
}

void JerkEstimator::Clear() {
  speed_filter_.Clear();
  jerk_filter_.Clear();
  last_accept_time_ = std::nullopt;
  t_hist_[0] = t_hist_[1] = t_hist_[2] = 0.0;
  v_hist_[0] = v_hist_[1] = v_hist_[2] = 0.0;
  vel_points_ = 0;
  last_jerk_ = std::nullopt;
  last_filtered_speed_ = 0.0;
  max_abs_jerk_ = 0.0;
}
