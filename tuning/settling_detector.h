#ifndef TUNING_SETTLING_DETECTOR_H_
#define TUNING_SETTLING_DETECTOR_H_

#include <cstddef>
#include <limits>

#include "statistics/WelfordAlgorithm.h"

// Detects that a closed-loop step response has reached its steady state.
//
// The response is "settled" when, for the whole hold window, the output has
// stayed inside the settling band around the reference AND its standard
// deviation over that window is below the band. The band is derived from the
// plant's noise estimate with a floor relative to the reference, so a slow
// ramp (small std-dev but far from the reference) is never mistaken for
// steady state, and noise-level wobble around the reference is tolerated.
class SettlingDetector {
 public:
  struct Config {
    double dt{0.001};
    double reference{0.0};
    double holdS{5.0};               // time the output must stay in band
    double bandFrac{0.02};           // band = max(bandFrac*|ref|, noiseMult*noise)
    double noiseMult{5.0};
    double noise{0.0};
    double minBandAbs{1e-9};
  };

  explicit SettlingDetector(const Config& config);

  // Feeds one output sample; returns true once settled (stays true).
  bool update(double output);
  void reset();

  bool settled() const { return _settled; }
  double band() const { return _band; }
  // Seconds since the first sample of the final in-band stretch.
  double inBandS() const { return static_cast<double>(_inBandSteps) * _config.dt; }
  // Time (s, from the first sample) at which the output entered the band for
  // the last time; NaN before the first entry.
  double lastBandEntryTimeS() const;
  // Std-dev of the output over the hold window (0 until the window is full).
  double windowDeviation() { return _window.GetDeviation(); }
  // Settling time: time at which the output entered the band for good. Valid
  // once settled().
  double settlingTimeS() const { return lastBandEntryTimeS(); }

  static double computeBand(const Config& config);

 private:
  Config _config;
  double _band;
  std::size_t _holdSteps;
  WelfordAlgorithm _window;
  std::size_t _sampleCount{0};
  std::size_t _inBandSteps{0};
  std::size_t _lastBandEntryIndex{std::numeric_limits<std::size_t>::max()};
  bool _settled{false};
};

#endif  // TUNING_SETTLING_DETECTOR_H_
