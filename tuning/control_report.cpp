#include "control_report.h"

#include <cmath>
#include <iomanip>
#include <sstream>

// ---------------------------------------------------------------------------
// PhaseTimer
// ---------------------------------------------------------------------------

void PhaseTimer::step(const std::string& phaseName) {
  if (phaseName != _currentName) {
    finishCurrent();
    _currentName = phaseName;
    _currentStart = Clock::now();
    _currentSteps = 0;
  }
  ++_currentSteps;
}

void PhaseTimer::finishCurrent() {
  if (_currentName.empty()) {
    return;
  }
  PhaseTiming t;
  t.name = _currentName;
  t.wallS = std::chrono::duration<double>(Clock::now() - _currentStart).count();
  t.steps = _currentSteps;
  t.simS = static_cast<double>(_currentSteps) * _dt;
  _phases.push_back(t);
  _currentName.clear();
  _currentSteps = 0;
}

std::vector<PhaseTiming> PhaseTimer::phases() {
  finishCurrent();
  return _phases;
}

PhaseTiming PhaseTimer::total() {
  finishCurrent();
  PhaseTiming t;
  t.name = "Total";
  t.wallS = std::chrono::duration<double>(Clock::now() - _runStart).count();
  for (const PhaseTiming& p : _phases) {
    t.simS += p.simS;
    t.steps += p.steps;
  }
  return t;
}

// ---------------------------------------------------------------------------
// Report
// ---------------------------------------------------------------------------

namespace {

std::string fmt(double v, int precision = 3) {
  if (std::isnan(v)) {
    return "n/a";
  }
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(precision) << v;
  return oss.str();
}

std::string seconds(double v, int precision = 3) {
  return std::isnan(v) ? "n/a" : fmt(v, precision) + " s";
}

std::string percent(double v, int precision = 2) {
  return std::isnan(v) ? "n/a" : fmt(v, precision) + " %";
}

void line(std::ostream& os, const std::string& label, const std::string& value) {
  os << "  " << std::left << std::setw(30) << label << value << "\n";
}

}  // namespace

void printControlReport(std::ostream& os, const ControlReport& r) {
  const StepResponseMetrics& m = r.metrics;
  const std::string rule(64, '=');
  const std::string thin(64, '-');

  os << "\n" << rule << "\n"
     << "                      FINAL CONTROL REPORT\n"
     << rule << "\n";
  line(os, "FMU", r.fmuPath);
  line(os, "Trace file", r.tracePath);
  if (!r.livePlotUrl.empty()) {
    line(os, "Live plot", r.livePlotUrl);
  }
  line(os, "Simulation step", seconds(r.dt, 4));

  os << thin << "\n" << "Identified plant (Sundaresan-Krishnaswamy, FOPDT)\n";
  line(os, "Gain K", fmt(r.plant.K, 4));
  line(os, "Time constant tau", seconds(r.plant.tau));
  line(os, "Dead time theta", seconds(r.plant.theta));
  line(os, "Output noise estimate", fmt(r.noise, 6));

  os << thin << "\n" << "Tuned controller (Skogestad SIMC, tau_c = " << fmt(r.tauC, 2) << " s)\n";
  line(os, "Kp", fmt(r.pid.Kp, 6));
  line(os, "Ti", seconds(r.pid.Ti, 4));
  line(os, "Td", seconds(r.pid.Td, 4));

  os << thin << "\n"
     << "Closed-loop validation: step 0 -> " << fmt(m.reference, 2)
     << " (band +/-" << fmt(m.settlingBand, 3) << ", horizon " << fmt(r.validationMaxDurationS, 0)
     << " s max, settle hold " << fmt(r.settleHoldS, 0) << " s)\n";
  line(os, "Simulated", seconds(m.horizonS, 3) + " (" + std::to_string(m.steps) + " steps)");
  line(os, "Rise time (10-90%)", seconds(m.riseTimeS));
  line(os, "Peak", fmt(m.peakValue, 4) + " at " + seconds(m.peakTimeS));
  line(os, "Overshoot", percent(m.overshootPercent));
  line(os, "Settling time (in band)",
       m.settled ? seconds(m.settlingTimeS)
                 : "NOT SETTLED within " + seconds(m.horizonS, 1));
  line(os, "Oscillations", std::to_string(m.oscillations));
  line(os, "Final value", fmt(m.finalValue, 4));
  line(os, "Steady-state error",
       fmt(m.steadyStateError, 4) + " (" + percent(m.steadyStateErrorPercent) + ")");
  line(os, "IAE", fmt(m.iae, 4));
  line(os, "ISE", fmt(m.ise, 4));
  line(os, "Max |jerk|", fmt(m.jerkAbsMax, 4));

  os << thin << "\n" << "Execution time\n";
  for (const PhaseTiming& p : r.phases) {
    std::ostringstream v;
    v << std::fixed << std::setprecision(2) << p.wallS << " s wall, " << std::setprecision(1)
      << p.simS << " s sim (" << p.steps << " steps)";
    line(os, p.name, v.str());
  }
  {
    std::ostringstream v;
    v << std::fixed << std::setprecision(2) << r.total.wallS << " s wall, " << std::setprecision(1)
      << r.total.simS << " s sim (" << r.total.steps << " steps)";
    if (r.total.wallS > 0.0) {
      v << ", " << std::setprecision(1) << (r.total.simS / r.total.wallS) << "x real time";
    }
    line(os, "Total", v.str());
  }
  os << rule << "\n";
}
