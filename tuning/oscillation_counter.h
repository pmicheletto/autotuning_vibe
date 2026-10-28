#ifndef TUNING_OSCILLATION_COUNTER_H_
#define TUNING_OSCILLATION_COUNTER_H_

// Counts oscillations of a control error around zero.
//
// An excursion outside the deadband (|e| > band) is a "lobe" once it has lasted
// minOutsideSteps samples; the band is re-entered with hysteresis
// (|e| < band * releaseRatio). Each confirmed lobe whose sign differs from the
// previous confirmed lobe counts as one oscillation, so a single overshoot is
// not an oscillation but overshoot followed by undershoot is.
class OscillationCounter {
 public:
  OscillationCounter(double band, unsigned minOutsideSteps, double releaseRatio = 0.58);

  void update(double error);
  void reset();

  int count() const { return _count; }

  // Minimum lobe duration used by the tuning tests: max(minSteps, minSeconds / dt).
  static unsigned minOutsideSteps(double dt, unsigned minSteps = 18, double minSeconds = 0.080);

 private:
  double _band;
  double _release;
  unsigned _minOutsideSteps;
  int _outside{0};        // hysteresis state: -1, 0, +1
  int _streakDir{0};
  unsigned _streak{0};
  bool _lobeConfirmed{false};
  int _lastLobeSign{0};
  int _count{0};
};

#endif  // TUNING_OSCILLATION_COUNTER_H_
