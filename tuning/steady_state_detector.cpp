#include "steady_state_detector.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

SteadyStateDetector::SteadyStateDetector(const Config& config)
    : _config(config),
      _n(static_cast<std::size_t>(std::ceil(config.windowS / config.dt))),
      _holdSteps(static_cast<std::size_t>(std::ceil(config.holdS / config.dt))),
      _ring(2 * std::max<std::size_t>(1, _n), 0.0) {
  if (config.dt <= 0.0 || config.windowS <= 0.0 || config.holdS < 0.0 || config.relTol < 0.0 ||
      config.absTol < 0.0) {
    throw std::invalid_argument("SteadyStateDetector: invalid configuration");
  }
  _n = std::max<std::size_t>(1, _n);
}

void SteadyStateDetector::reset() {
  std::fill(_ring.begin(), _ring.end(), 0.0);
  _head = 0;
  _count = 0;
  _sumCur = _sumPrev = 0.0;
  _passes = 0;
  _steady = false;
}

bool SteadyStateDetector::update(double value) {
  const std::size_t size = _ring.size();
  // Sample leaving the current window moves to the previous one; the sample
  // leaving the previous window is dropped.
  const double leavingCur = _ring[(_head + size - _n) % size];
  const double leavingPrev = _ring[_head];  // oldest sample, overwritten below
  if (_count >= _n) {
    _sumCur -= leavingCur;
    _sumPrev += leavingCur;
  }
  if (_count >= 2 * _n) {
    _sumPrev -= leavingPrev;
  }
  _ring[_head] = value;
  _head = (_head + 1) % size;
  _sumCur += value;
  if (_count < 2 * _n) {
    ++_count;
  }

  if (!windowsFull()) {
    return false;
  }
  const bool pass = std::fabs(mean() - previousMean()) <= tolerance();
  _passes = pass ? _passes + 1 : 0;
  if (!_steady && _passes >= std::max<std::size_t>(1, _holdSteps)) {
    _steady = true;
  }
  return _steady;
}

double SteadyStateDetector::mean() const {
  return (_count >= _n) ? _sumCur / static_cast<double>(_n) : 0.0;
}

double SteadyStateDetector::previousMean() const {
  return windowsFull() ? _sumPrev / static_cast<double>(_n) : 0.0;
}

double SteadyStateDetector::deviation() const {
  // Two-pass over the current window: exact, and only evaluated on demand
  // (a running sum-of-squares loses all precision for large offsets).
  if (_count < _n || _n < 2) {
    return 0.0;
  }
  const std::size_t size = _ring.size();
  const double m = mean();
  double ss = 0.0;
  for (std::size_t k = 1; k <= _n; ++k) {
    const double d = _ring[(_head + size - k) % size] - m;
    ss += d * d;
  }
  return std::sqrt(ss / static_cast<double>(_n - 1));
}

double SteadyStateDetector::tolerance() const {
  return std::max(_config.absTol, _config.relTol * std::fabs(mean()));
}
