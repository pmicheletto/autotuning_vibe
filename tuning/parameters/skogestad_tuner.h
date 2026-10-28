#ifndef TUNING_SKOGESTAD_TUNER_H_
#define TUNING_SKOGESTAD_TUNER_H_

#include "../models.h"

// Skogestad SIMC rules for a FOPDT model, with tau_c the desired closed-loop
// time constant:
//   Kp = (tau + theta/2) / (K (tau_c + theta/2))
//   Ti = min(tau + theta/2, 4 (tau_c + theta/2))
//   Td = tau theta / (2 tau + theta)     (PID only)
// All-zero parameters are returned for a degenerate model (K = 0 or
// tau_c + theta/2 <= 0); ParameterValidator rejects those.
class SkogestadTuner {
 public:
  static PIDParameters calculatePI(const PlantModel& model, double tc);
  static PIDParameters calculatePID(const PlantModel& model, double tc);
};

#endif  // TUNING_SKOGESTAD_TUNER_H_
