#ifndef TUNING_TRANSFER_FUNCTION_SUNDARESAN_H_
#define TUNING_TRANSFER_FUNCTION_SUNDARESAN_H_

#include "../models.h"
#include "../steady_state_detector.h"
#include "../system/system_interface.h"
#include "statistics/WelfordAlgorithm.h"

// Open-loop FOPDT identification (Sundaresan-Krishnaswamy two-point method).
//
//   1. Probe: raise the manual step until the plant moves, let it settle and
//      measure the output noise at that plateau.
//   2. From rest, apply a 0.5 step; when the response has settled, its level
//      is the final value y_inf and the noise there is the plant noise used by
//      the closed-loop tests.
//   3. From rest again, re-apply the step and time t1 (35.3% of y_inf) and
//      t2 (85.3%):  tau = 0.67 (t2 - t1),  theta = 1.3 t1 - 0.29 t2,
//      K = y_inf / step.
//
// Drive it with one Execute() per simulation step (kSimulationDt).
enum class SundaresanState {
  FindingMovement,
  FindingDeadBand,
  CalculateTransferFunction,
  Done,
  UnresponsivePlant,
};

class Sundaresan {
 public:
  explicit Sundaresan(ISystem& system);

  // Advances the state machine by one simulation step.
  void Execute();

  [[nodiscard]] bool IsFinished() const;
  [[nodiscard]] SundaresanState GetState() const { return _state; }
  [[nodiscard]] const PlantModel& GetPlantModel() const { return _plantModel; }
  [[nodiscard]] double GetFinalValue() const { return _finalValue; }
  // Output std-dev at the first-step plateau (valid once Done).
  [[nodiscard]] double GetNoise() const { return _noise; }

 private:
  enum class CalcPhase {
    WaitZeroBeforeStep,
    DetectTransientStart,
    WaitSettled,
    WaitZeroAfterFirstRun,
    MeasureT1T2,
  };

  double readFilteredSpeed();
  void checkThereIsMovement();
  void checkDeadBand();
  void runCalculateTransferFunction();
  void runPhaseWaitZero(double filteredSpeed, CalcPhase next);
  void runPhaseDetectTransientStart();
  void runPhaseWaitSettled(double filteredSpeed);
  void runPhaseMeasureT1T2(double filteredSpeed);
  void finishPlantModel();

  ISystem* _system;
  SundaresanState _state{SundaresanState::FindingMovement};
  PlantModel _plantModel;
  double _step;                 // current probe step height
  int _movementTimeout;         // steps left before the probe step is raised
  unsigned int _stepCount{0};   // steps in the current phase
  double _finalValue{0.0};
  double _noise{0.0};
  double _probeNoise{0.0};
  SteadyStateDetector _steady;
  WelfordAlgorithm _speedFilter{10};

  CalcPhase _calcPhase{CalcPhase::WaitZeroBeforeStep};
  double _transientBaseline{0.0};
  double _t1{0.0};
  double _t2{0.0};
  bool _t1Set{false};
  bool _t2Set{false};
  bool _secondStepApplied{false};
};

#endif  // TUNING_TRANSFER_FUNCTION_SUNDARESAN_H_
