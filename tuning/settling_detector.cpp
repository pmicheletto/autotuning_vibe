#include "settling_detector.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

double SettlingDetector::computeBand(const Config& c) {
  return std::max({c.bandFrac * std::fabs(c.reference), c.noiseMult * c.noise, c.minBandAbs});
}

SettlingDetector::SettlingDetector(const Config& config)
    : _config(config),
      _band(computeBand(config)),
      _holdSteps(static_cast<std::size_t>(std::ceil(config.holdS / config.dt))),
      _window(static_cast<unsigned int>(std::max<std::size_t>(2, _holdSteps))) {
  if (config.dt <= 0.0 || config.holdS <= 0.0) {
    throw std::invalid_argument("SettlingDetector: dt and holdS must be > 0");
  }
}

void SettlingDetector::reset() {
  _window.Clear();
  _sampleCount = 0;
  _inBandSteps = 0;
  _lastBandEntryIndex = std::numeric_limits<std::size_t>::max();
  _settled = false;
}

bool SettlingDetector::update(double output) {
  _window.AddSample(output);
  const bool inBand = std::isfinite(output) && std::fabs(output - _config.reference) <= _band;
  if (inBand) {
    if (_inBandSteps == 0) {
      _lastBandEntryIndex = _sampleCount;
    }
    ++_inBandSteps;
  } else {
    _inBandSteps = 0;
  }
  ++_sampleCount;

  if (!_settled && _inBandSteps >= _holdSteps && _window.GetDeviation() <= _band) {
    _settled = true;
  }
  return _settled;
}

double SettlingDetector::lastBandEntryTimeS() const {
  if (_lastBandEntryIndex == std::numeric_limits<std::size_t>::max()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return static_cast<double>(_lastBandEntryIndex + 1) * _config.dt;
}
