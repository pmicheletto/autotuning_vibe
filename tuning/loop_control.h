#ifndef TUNING_LOOP_CONTROL_H_
#define TUNING_LOOP_CONTROL_H_

#include "system/system_interface.h"

// Plant-side switches and controller parameters of the Modelica driver model.
namespace loop_control {

// Manual (open loop): the reference step drives the pedal directly.
void openLoop(ISystem& system);
// Automatic (closed loop): the PID drives the pedal from the reference step.
void closeLoop(ISystem& system);

// Sets the driver PID (ideal form: Kp, Ti, Td) including the derived
// OpenModelica block parameters that do not update automatically after
// initialisation.
void applyPID(ISystem& system, double Kp, double Ti, double Td);

}  // namespace loop_control

#endif  // TUNING_LOOP_CONTROL_H_
