#ifndef STATISTICS_JERKESTIMATOR_H_
#define STATISTICS_JERKESTIMATOR_H_

#include <optional>

#include "WelfordAlgorithm.h"

// Jerk from raw speed: subsample, moving-average speed, two-step acceleration, then
// moving-average jerk (reduces noise/spikes).
class JerkEstimator {
 public:
  explicit JerkEstimator(unsigned int speed_mean_window = 20,
                         double min_sample_interval_sec = 0.05,
                         unsigned int jerk_mean_window = 20);
  JerkEstimator(const JerkEstimator&) = delete;
  JerkEstimator& operator=(const JerkEstimator&) = delete;
  ~JerkEstimator() = default;

  // Accepts samples at least min_sample_interval_sec_ apart. Returns filtered jerk when
  // enough history exists; nullopt if decimated or jerk not yet defined.
  std::optional<double> AddSpeed(double raw_speed, double time_sec);

  void Clear();

  std::optional<double> GetLastJerk() const { return last_jerk_; }
  double GetLastFilteredSpeed() const { return last_filtered_speed_; }
  double GetMaxAbsJerk() const { return max_abs_jerk_; }
  void ResetMaxAbsJerk() { max_abs_jerk_ = 0.0; }

 private:
  bool ShouldAcceptSample(double time_sec) const;

  WelfordAlgorithm speed_filter_;
  WelfordAlgorithm jerk_filter_;
  double min_sample_interval_sec_;
  std::optional<double> last_accept_time_;

  double t_hist_[3];
  double v_hist_[3];
  unsigned int vel_points_;

  std::optional<double> last_jerk_;
  double last_filtered_speed_;
  double max_abs_jerk_;
};

#endif  // STATISTICS_JERKESTIMATOR_H_
