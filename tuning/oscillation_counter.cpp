#include "oscillation_counter.h"

#include <algorithm>
#include <cmath>

OscillationCounter::OscillationCounter(double band, unsigned minOutsideSteps, double releaseRatio)
    : _band(std::max(band, 1e-9)),
      _release(std::max(_band * releaseRatio, 1e-9)),
      _minOutsideSteps(std::max(minOutsideSteps, 1u)) {}

unsigned OscillationCounter::minOutsideSteps(double dt, unsigned minSteps, double minSeconds) {
  const unsigned fromTime = (dt > 0.0) ? static_cast<unsigned>(std::ceil(minSeconds / dt)) : 0u;
  return std::max(minSteps, fromTime);
}

void OscillationCounter::reset() {
  _outside = 0;
  _streakDir = 0;
  _streak = 0;
  _lobeConfirmed = false;
  _lastLobeSign = 0;
  _count = 0;
}

void OscillationCounter::update(double error) {
  if (error > _band) {
    _outside = 1;
  } else if (error < -_band) {
    _outside = -1;
  } else if (std::fabs(error) < _release) {
    _outside = 0;
  }

  if (_outside == 0) {
    _streak = 0;
    _streakDir = 0;
    _lobeConfirmed = false;
    return;
  }
  if (_outside == _streakDir) {
    ++_streak;
  } else {
    _streakDir = _outside;
    _streak = 1;
    _lobeConfirmed = false;
  }
  if (!_lobeConfirmed && _streak >= _minOutsideSteps) {
    if (_lastLobeSign != 0 && _lastLobeSign != _streakDir) {
      ++_count;
    }
    _lastLobeSign = _streakDir;
    _lobeConfirmed = true;
  }
}
