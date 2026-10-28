#include "loop_control.h"

#include <algorithm>
#include <exception>

namespace loop_control {

namespace {

constexpr const char* kManualVar = "driver.useManualControl";
constexpr double kDerivativeFilterRatioFallback = 10.0;  // LimPID Nd
constexpr double kDerivativeTimeFloor = 2.220446049250313e-14;

}  // namespace

void openLoop(ISystem& system) { system.setVariable(kManualVar, 1.0); }

void closeLoop(ISystem& system) { system.setVariable(kManualVar, 0.0); }

void applyPID(ISystem& system, double Kp, double Ti, double Td) {
  system.setVariable("driver.pid.k", Kp);
  system.setVariable("driver.pid.Ti", Ti);
  system.setVariable("driver.pid.Td", Td);

  // OpenModelica's LimPID derives its internal block gains from k/Ti/Td only at
  // initialisation; mirror them explicitly. Optional: models without these
  // internals (e.g. test plants) simply ignore the writes.
  double nd = kDerivativeFilterRatioFallback;
  try {
    const double v = system.readValue("driver.pid.Nd");
    if (v > 1e-300) {
      nd = v;
    }
  } catch (const std::exception&) {
  }
  try {
    system.setVariable("driver.pid.Gain.k", Kp);
    system.setVariable("driver.pid.I.k", (Ti > 1e-12) ? 1.0 / Ti : 0.0);
    system.setVariable("driver.pid.D.k", Td);
    system.setVariable("driver.pid.D.T", std::max(Td / nd, kDerivativeTimeFloor));
    system.setVariable("driver.pid.P.k", 1.0);
  } catch (const std::exception&) {
  }
}

}  // namespace loop_control
