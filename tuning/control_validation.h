#ifndef TUNING_CONTROL_VALIDATION_H_
#define TUNING_CONTROL_VALIDATION_H_

#include <functional>
#include <limits>
#include <optional>
#include <vector>

#include "models.h"
#include "system/system_interface.h"

// Closed-loop step-response validation of a tuned controller.
//
// Runs the same procedure as the tuning candidate tests (open loop, wait for
// rest, apply PID, close loop, apply a reference step) but over a horizon long
// enough to measure the standard step-response metrics.

struct StepResponseMetrics {
  static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

  double reference{0.0};
  double settlingBand{0.0};     // half-width of the settling band, output units
  double horizonS{0.0};         // simulated seconds after the step
  unsigned long steps{0};

  double riseTimeS{kNaN};       // 10% -> 90% of reference
  double peakValue{kNaN};
  double peakTimeS{kNaN};
  double overshootPercent{0.0}; // 0 if the peak never exceeds the reference
  bool settled{false};          // stayed inside the band for >= settleHoldS at the end
  double settlingTimeS{kNaN};   // last entry into the band (NaN if not settled)
  int oscillations{0};          // alternating excursions outside the band after first touch
  double finalValue{kNaN};
  double steadyStateError{kNaN};        // reference - mean(output) over the final window
  double steadyStateErrorPercent{kNaN};
  double iae{kNaN};             // integral |e| dt over the horizon
  double ise{kNaN};             // integral e^2 dt over the horizon
  double jerkAbsMax{kNaN};      // from JerkEstimator at the configured cadence
};

struct ClosedLoopStepTestConfig {
  double dt{0.001};
  double reference{0.0};
  double maxDurationS{120.0};   // validation horizon after the step
  double settleHoldS{5.0};      // time inside the band required to call it settled
  double settlingBandFrac{0.02};
  double settlingBandNoiseMult{5.0};
  double steadyStateWindowS{1.0};
  double noise{0.0};            // output noise estimate (from identification)

  // Open-loop "prepare" phase before the step (mirrors the tuning tests).
  unsigned prepareStableSteps{80};
  unsigned prepareMaxSteps{250000};
  double prepareZeroAbsTol{5e-5};
  double prepareZeroNoiseMult{2.0};

  // Jerk estimation cadence/windows (same as the live UDP pipeline).
  unsigned jerkEveryNSteps{100};
  unsigned jerkSpeedMeanWindow{20};
  double jerkMinSampleIntervalS{0.05};
  unsigned jerkMeanWindow{20};
};

// One simulation step of the validation run, for live streaming/logging.
struct ClosedLoopSample {
  unsigned long index{0};       // 1-based step index since the reference step
  double timeS{0.0};            // seconds since the reference step
  double stepHeight{0.0};
  double output{0.0};
  std::optional<double> jerk;   // present only on jerk-cadence steps
  double jerkAbsMax{0.0};
};

using ClosedLoopSampleCallback = std::function<void(const ClosedLoopSample&)>;

// Pure metric computation from an output trace sampled every `dt` seconds,
// starting one step after the reference step was applied.
StepResponseMetrics computeStepResponseMetrics(const std::vector<double>& output, double dt,
                                               double reference, double settlingBand,
                                               double settleHoldS, double steadyStateWindowS);

// Executes the validation run on `system` and returns the metrics.
StepResponseMetrics runClosedLoopStepTest(ISystem& system, const PIDParameters& pid,
                                          const ClosedLoopStepTestConfig& config,
                                          const ClosedLoopSampleCallback& onSample = {});

#endif  // TUNING_CONTROL_VALIDATION_H_
