#ifndef STATISTICS_WELFORDALGORITHM_H_
#define STATISTICS_WELFORDALGORITHM_H_

#include <vector>

// Sliding-window mean and (sample) variance over the last `window_size` values.
class WelfordAlgorithm {
 public:
  explicit WelfordAlgorithm(unsigned int window_size = 10);

  void AddSample(double new_value);
  void Clear();
  double GetMean() const { return mean_; }
  // 0 until the window is full.
  double GetVariance() const;
  double GetDeviation() const;

 private:
  std::vector<double> window_;
  unsigned int count_{0};
  unsigned int window_index_{0};
  double mean_{0.0};
  double m2_{0.0};  // sum of squared distances from the mean
  double sum_{0.0};
  double sum_squares_{0.0};
};

#endif  // STATISTICS_WELFORDALGORITHM_H_
