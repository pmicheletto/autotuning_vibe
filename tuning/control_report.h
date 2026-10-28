#ifndef TUNING_CONTROL_REPORT_H_
#define TUNING_CONTROL_REPORT_H_

#include <chrono>
#include <ostream>
#include <string>
#include <vector>

#include "control_validation.h"
#include "models.h"

// Wall-clock vs. simulated time bookkeeping for one phase of the run.
struct PhaseTiming {
  std::string name;
  double wallS{0.0};
  double simS{0.0};
  unsigned long steps{0};
};

// Accumulates per-phase timings; `step()` is called once per simulation step
// with the name of the phase it belongs to.
class PhaseTimer {
 public:
  explicit PhaseTimer(double dt) : _dt(dt) {}

  void step(const std::string& phaseName);
  void add(const PhaseTiming& timing) { finishCurrent(); _phases.push_back(timing); }

  std::vector<PhaseTiming> phases();
  PhaseTiming total();

 private:
  void finishCurrent();

  using Clock = std::chrono::steady_clock;
  double _dt;
  std::vector<PhaseTiming> _phases;
  std::string _currentName;
  Clock::time_point _currentStart{};
  unsigned long _currentSteps{0};
  Clock::time_point _runStart{Clock::now()};
};

struct ControlReport {
  std::string fmuPath;
  std::string tracePath;
  std::string livePlotUrl;  // empty when the live plot is not running
  PlantModel plant;
  double noise{0.0};
  PIDParameters pid;
  double tauC{0.0};
  double dt{0.0};
  double validationMaxDurationS{0.0};
  double settleHoldS{0.0};
  StepResponseMetrics metrics;
  std::vector<PhaseTiming> phases;
  PhaseTiming total;
};

void printControlReport(std::ostream& os, const ControlReport& report);

#endif  // TUNING_CONTROL_REPORT_H_
