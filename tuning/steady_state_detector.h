#ifndef TUNING_STEADY_STATE_DETECTOR_H_
#define TUNING_STEADY_STATE_DETECTOR_H_

#include <cstddef>
#include <vector>

// Open-loop steady-state detector for a response whose final value is not
// known in advance (plant identification).
//
// Compares the mean of the most recent window with the mean of the window
// before it: the output has reached steady state when that difference stays
// below a tolerance for a full hold window. The tolerance is relative to the
// current level (plus an absolute floor), so the criterion works at any
// operating point and is immune to the sampling rate. A window long enough to
// average out the plant's own wobble makes noise-level variation irrelevant,
// while a slow but steady rise keeps the two means apart and is never mistaken
// for steady state.
class SteadyStateDetector {
 public:
  struct Config {
    double dt{0.001};
    double windowS{5.0};    // length of each of the two compared windows
    double holdS{5.0};      // how long the condition must hold
    double relTol{0.01};    // |mean_now - mean_prev| <= relTol * |mean_now| ...
    double absTol{1e-3};    // ... or <= absTol, whichever is larger
  };

  explicit SteadyStateDetector(const Config& config);

  // Feeds one sample; returns true once steady (stays true until reset()).
  bool update(double value);
  void reset();

  bool steady() const { return _steady; }
  bool windowsFull() const { return _count >= 2 * _n; }
  // Mean / std-dev of the most recent window (0 until it is full).
  double mean() const;
  double deviation() const;
  // Mean of the window before the most recent one (0 until full).
  double previousMean() const;
  double tolerance() const;
  std::size_t samples() const { return _count; }

 private:
  Config _config;
  std::size_t _n;            // samples per window
  std::size_t _holdSteps;
  std::vector<double> _ring; // 2 * _n samples
  std::size_t _head{0};      // next write position
  std::size_t _count{0};
  double _sumCur{0.0}, _sumPrev{0.0};
  std::size_t _passes{0};
  bool _steady{false};
};

#endif  // TUNING_STEADY_STATE_DETECTOR_H_
