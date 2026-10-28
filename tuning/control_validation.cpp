#include "control_validation.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "loop_control.h"
#include "oscillation_counter.h"
#include "settling_detector.h"
#include "statistics/JerkEstimator.h"

namespace {

constexpr const char* kOutputVar = "out";
constexpr const char* kStepHeightVar = "driver.step.height";

// Oscillations are counted from the first entry into the band onwards.
int countOscillations(const std::vector<double>& output, double dt, double reference, double band) {
  OscillationCounter counter(band, OscillationCounter::minOutsideSteps(dt));
  bool touched = false;
  for (double y : output) {
    const double e = y - reference;
    touched = touched || std::fabs(e) <= band;
    if (touched) {
      counter.update(e);
    }
  }
  return counter.count();
}

}  // namespace

StepResponseMetrics computeStepResponseMetrics(const std::vector<double>& output, double dt,
                                               double reference, double settlingBand,
                                               double settleHoldS, double steadyStateWindowS) {
  StepResponseMetrics m;
  m.reference = reference;
  m.settlingBand = settlingBand;
  m.steps = output.size();
  m.horizonS = static_cast<double>(output.size()) * dt;
  if (output.empty() || dt <= 0.0) {
    return m;
  }
  const auto timeAt = [dt](std::size_t i) { return static_cast<double>(i + 1) * dt; };
  const double direction = reference >= 0.0 ? 1.0 : -1.0;  // support negative references

  // Rise time 10% -> 90% (measured along the step direction).
  std::optional<double> t10, t90;
  for (std::size_t i = 0; i < output.size(); ++i) {
    const double progress = direction * output[i];
    if (!t10 && progress >= 0.1 * std::fabs(reference)) t10 = timeAt(i);
    if (!t90 && progress >= 0.9 * std::fabs(reference)) {
      t90 = timeAt(i);
      break;
    }
  }
  if (t10 && t90) {
    m.riseTimeS = *t90 - *t10;
  }

  // Peak and overshoot.
  std::size_t peakIdx = 0;
  for (std::size_t i = 1; i < output.size(); ++i) {
    if (direction * output[i] > direction * output[peakIdx]) {
      peakIdx = i;
    }
  }
  m.peakValue = output[peakIdx];
  m.peakTimeS = timeAt(peakIdx);
  if (std::fabs(reference) > 1e-12 && direction * (m.peakValue - reference) > 0.0) {
    m.overshootPercent = 100.0 * std::fabs(m.peakValue - reference) / std::fabs(reference);
  }

  // Settling: last sample outside the band; settled only if the trailing
  // inside-band stretch is at least settleHoldS long.
  std::size_t lastOutside = output.size();  // "none"
  for (std::size_t i = output.size(); i-- > 0;) {
    if (std::fabs(output[i] - reference) > settlingBand) {
      lastOutside = i;
      break;
    }
  }
  const std::size_t firstInsideForGood = (lastOutside == output.size()) ? 0 : lastOutside + 1;
  const double trailingInsideS = static_cast<double>(output.size() - firstInsideForGood) * dt;
  if (firstInsideForGood < output.size() && trailingInsideS >= settleHoldS) {
    m.settled = true;
    m.settlingTimeS = (firstInsideForGood == 0) ? 0.0 : timeAt(firstInsideForGood);
  }

  m.oscillations = countOscillations(output, dt, reference, settlingBand);

  // Steady state over the final window.
  const std::size_t window = std::min<std::size_t>(
      output.size(), std::max<std::size_t>(1, static_cast<std::size_t>(steadyStateWindowS / dt)));
  double sum = 0.0;
  for (std::size_t i = output.size() - window; i < output.size(); ++i) {
    sum += output[i];
  }
  const double finalMean = sum / static_cast<double>(window);
  m.finalValue = output.back();
  m.steadyStateError = reference - finalMean;
  m.steadyStateErrorPercent =
      std::fabs(reference) > 1e-12 ? 100.0 * m.steadyStateError / std::fabs(reference)
                                   : StepResponseMetrics::kNaN;

  // Integral error indices.
  double iae = 0.0, ise = 0.0;
  for (double y : output) {
    const double e = reference - y;
    iae += std::fabs(e) * dt;
    ise += e * e * dt;
  }
  m.iae = iae;
  m.ise = ise;
  return m;
}

StepResponseMetrics runClosedLoopStepTest(ISystem& system, const PIDParameters& pid,
                                          const ClosedLoopStepTestConfig& config,
                                          const ClosedLoopSampleCallback& onSample) {
  if (config.dt <= 0.0) {
    throw std::invalid_argument("runClosedLoopStepTest: dt must be > 0");
  }

  // Prepare: fresh FMU instance, open loop, reference 0, wait for rest.
  system.reset();
  loop_control::openLoop(system);
  system.setVariable(kStepHeightVar, 0.0);
  const double restTol =
      std::max(config.prepareZeroAbsTol, config.prepareZeroNoiseMult * config.noise);
  unsigned restStreak = 0;
  for (unsigned i = 0; i < config.prepareMaxSteps && restStreak < config.prepareStableSteps; ++i) {
    system.advanceSimulation(config.dt);
    restStreak = (std::fabs(system.readValue(kOutputVar)) <= restTol) ? restStreak + 1 : 0;
  }

  // Apply the tuned controller and the reference step.
  loop_control::applyPID(system, pid.Kp, pid.Ti, pid.Td);
  loop_control::closeLoop(system);
  system.setVariable(kStepHeightVar, config.reference);

  SettlingDetector::Config settleCfg;
  settleCfg.dt = config.dt;
  settleCfg.reference = config.reference;
  settleCfg.holdS = config.settleHoldS;
  settleCfg.bandFrac = config.settlingBandFrac;
  settleCfg.noiseMult = config.settlingBandNoiseMult;
  settleCfg.noise = std::max(config.noise, 1e-12);
  SettlingDetector settling(settleCfg);
  const double band = settling.band();
  const unsigned long maxSteps =
      static_cast<unsigned long>(std::ceil(config.maxDurationS / config.dt));

  JerkEstimator jerk(config.jerkSpeedMeanWindow, config.jerkMinSampleIntervalS,
                     config.jerkMeanWindow);
  std::vector<double> trace;
  trace.reserve(maxSteps);

  for (unsigned long i = 1; i <= maxSteps; ++i) {
    system.advanceSimulation(config.dt);
    const double t = static_cast<double>(i) * config.dt;
    const double y = system.readValue(kOutputVar);
    trace.push_back(y);

    ClosedLoopSample sample;
    sample.index = i;
    sample.timeS = t;
    sample.stepHeight = config.reference;
    sample.output = y;
    if (config.jerkEveryNSteps > 0 && i % config.jerkEveryNSteps == 0) {
      sample.jerk = jerk.AddSpeed(y, t);
    }
    sample.jerkAbsMax = jerk.GetMaxAbsJerk();
    if (onSample) {
      onSample(sample);
    }

    if (settling.update(y)) {
      break;  // settled: in band for the whole hold window, windowed std-dev below band
    }
  }

  StepResponseMetrics metrics = computeStepResponseMetrics(
      trace, config.dt, config.reference, band, config.settleHoldS, config.steadyStateWindowS);
  metrics.jerkAbsMax = jerk.GetMaxAbsJerk();
  return metrics;
}
