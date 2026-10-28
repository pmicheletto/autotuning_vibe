#include "sundaresan.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace {

constexpr const char* kOutputVar = "out";
constexpr const char* kStepVar = "driver.step.height";
constexpr double kSimulationDt = 0.001;

constexpr double kPercentT1 = 0.353;
constexpr double kPercentT2 = 0.853;
constexpr double kIdentificationStep = 0.5;

// Probe: start at kProbeStepStart, add kProbeStepIncrement every
// kMovementTimeoutSteps without movement, give up beyond kProbeStepMax.
constexpr double kProbeStepStart = 0.01;
constexpr double kProbeStepIncrement = 1.0;
constexpr double kProbeStepMax = 50.0;
constexpr int kMovementTimeoutSteps = 5000;
constexpr double kMovementThreshold = 0.2;

// "At rest": filtered output below this for kMinStepsAtZero consecutive steps.
constexpr double kSpeedNearZeroThreshold = 0.1;
constexpr unsigned int kMinStepsAtZero = 500;

// Steady state (open loop): the mean of the last kSteadyWindowS equals the mean
// of the window before within kSteadyRelTol (or kSteadyAbsTol) for kSteadyHoldS.
constexpr double kSteadyWindowS = 5.0;
constexpr double kSteadyHoldS = 5.0;
constexpr double kSteadyRelTol = 0.01;
constexpr double kSteadyAbsTol = 1e-3;
// Safety limits: accept the current level if the plant keeps wandering.
constexpr double kProbeSettleMaxS = 120.0;
constexpr double kStepSettleMaxS = 300.0;
// Transient start: output moved from its pre-step level by more than this.
constexpr double kTransientNoiseMult = 5.0;
constexpr double kTransientAbsMin = 1e-3;

SteadyStateDetector::Config steadyConfig() {
  SteadyStateDetector::Config c;
  c.dt = kSimulationDt;
  c.windowS = kSteadyWindowS;
  c.holdS = kSteadyHoldS;
  c.relTol = kSteadyRelTol;
  c.absTol = kSteadyAbsTol;
  return c;
}

}  // namespace

Sundaresan::Sundaresan(ISystem& system)
    : _system(&system),
      _step(kProbeStepStart),
      _movementTimeout(kMovementTimeoutSteps),
      _steady(steadyConfig()) {
  std::cout << "[Sundaresan] Etapa: início — procurando movimento (degrau inicial = " << _step
            << ")\n";
}

bool Sundaresan::IsFinished() const {
  return _state == SundaresanState::Done || _state == SundaresanState::UnresponsivePlant;
}

void Sundaresan::Execute() {
  switch (_state) {
    case SundaresanState::FindingMovement:
      _system->setVariable(kStepVar, _step);
      _movementTimeout--;
      checkThereIsMovement();
      break;

    case SundaresanState::FindingDeadBand:
      _system->setVariable(kStepVar, _step);
      _stepCount++;
      checkDeadBand();
      break;

    case SundaresanState::CalculateTransferFunction: {
      const bool atRest = _calcPhase == CalcPhase::WaitZeroBeforeStep ||
                          _calcPhase == CalcPhase::WaitZeroAfterFirstRun;
      _system->setVariable(kStepVar, atRest ? 0.0 : kIdentificationStep);
      runCalculateTransferFunction();
      break;
    }

    case SundaresanState::Done:
    case SundaresanState::UnresponsivePlant:
      break;
  }
}

double Sundaresan::readFilteredSpeed() {
  _speedFilter.AddSample(_system->readValue(kOutputVar));
  return _speedFilter.GetMean();
}

void Sundaresan::checkThereIsMovement() {
  if (readFilteredSpeed() > kMovementThreshold) {
    std::cout << "[Sundaresan] Etapa: movimento detectado — medindo ruído no patamar de sondagem\n";
    _state = SundaresanState::FindingDeadBand;
    _stepCount = 0;
    _steady.reset();
    return;
  }
  if (_movementTimeout == 0) {
    _step += kProbeStepIncrement;
    _movementTimeout = kMovementTimeoutSteps;
    std::cout << "[Sundaresan] Etapa: tempo esgotado sem movimento — aumentando degrau para "
              << _step << "\n";
  }
  if (_step > kProbeStepMax) {
    std::cout << "[Sundaresan] Etapa: planta não responsiva (degrau além do limite)\n";
    _state = SundaresanState::UnresponsivePlant;
  }
}

void Sundaresan::checkDeadBand() {
  // Let the probe-step response reach its plateau, then measure the noise there.
  const bool steady = _steady.update(readFilteredSpeed());
  const bool timeout = _stepCount * kSimulationDt > kProbeSettleMaxS;
  if (!steady && !timeout) {
    return;
  }
  if (timeout) {
    std::cout << "Aviso: resposta ao degrau de sondagem nao estabilizou em " << kProbeSettleMaxS
              << " s — usando o nivel atual.\n";
  }
  _probeNoise = _steady.deviation();
  std::cout << "[Sundaresan] Patamar de sondagem = " << _steady.mean() << " após "
            << _stepCount * kSimulationDt << " s, ruído = " << _probeNoise
            << " — iniciando função de transferência\n";
  _speedFilter.Clear();
  _calcPhase = CalcPhase::WaitZeroBeforeStep;
  _stepCount = 0;
  _system->setVariable(kStepVar, 0.0);
  _state = SundaresanState::CalculateTransferFunction;
}

void Sundaresan::runCalculateTransferFunction() {
  const double filteredSpeed = readFilteredSpeed();
  switch (_calcPhase) {
    case CalcPhase::WaitZeroBeforeStep:
      runPhaseWaitZero(filteredSpeed, CalcPhase::DetectTransientStart);
      break;
    case CalcPhase::DetectTransientStart:
      runPhaseDetectTransientStart();
      break;
    case CalcPhase::WaitSettled:
      runPhaseWaitSettled(filteredSpeed);
      break;
    case CalcPhase::WaitZeroAfterFirstRun:
      runPhaseWaitZero(filteredSpeed, CalcPhase::MeasureT1T2);
      break;
    case CalcPhase::MeasureT1T2:
      runPhaseMeasureT1T2(filteredSpeed);
      break;
  }
}

// Waits until the output has been near zero for kMinStepsAtZero steps.
void Sundaresan::runPhaseWaitZero(double filteredSpeed, CalcPhase next) {
  if (filteredSpeed >= kSpeedNearZeroThreshold) {
    _stepCount = 0;
    return;
  }
  if (++_stepCount < kMinStepsAtZero) {
    return;
  }
  _stepCount = 0;
  _speedFilter.Clear();
  _calcPhase = next;
  if (next == CalcPhase::DetectTransientStart) {
    _transientBaseline = filteredSpeed;
    std::cout << "[Sundaresan] Etapa: saída em repouso — aplicando degrau, detectando início do transitório\n";
  } else {
    _secondStepApplied = false;
    _t1Set = false;
    _t2Set = false;
    std::cout << "[Sundaresan] Etapa: retorno ao repouso — preparando medição de t1 e t2\n";
  }
}

void Sundaresan::runPhaseDetectTransientStart() {
  // The step is applied now; wait (through the dead time) until the output has
  // clearly moved away from its pre-step level.
  const double threshold = std::max(kTransientNoiseMult * _probeNoise, kTransientAbsMin);
  if (std::fabs(_speedFilter.GetMean() - _transientBaseline) <= threshold) {
    return;
  }
  std::cout << "[Sundaresan] Etapa: transitório detectado — aguardando estabilização da resposta\n";
  _steady.reset();
  _stepCount = 0;
  _calcPhase = CalcPhase::WaitSettled;
}

void Sundaresan::runPhaseWaitSettled(double filteredSpeed) {
  _stepCount++;
  const bool steady = _steady.update(filteredSpeed);
  const bool timeout = _stepCount * kSimulationDt > kStepSettleMaxS;
  if (!steady && !timeout) {
    return;
  }
  if (timeout) {
    std::cout << "Aviso: resposta ao degrau nao estabilizou em " << kStepSettleMaxS
              << " s — usando o nivel atual.\n";
  }
  _finalValue = _steady.mean();
  _noise = _steady.deviation();
  std::cout << "[Sundaresan] Etapa: resposta estabilizada (" << _stepCount * kSimulationDt
            << " s) — valor final estimado = " << _finalValue << ", ruído no patamar = " << _noise
            << "\n";
  _calcPhase = CalcPhase::WaitZeroAfterFirstRun;
  _stepCount = 0;
  _system->setVariable(kStepVar, 0.0);
}

void Sundaresan::runPhaseMeasureT1T2(double filteredSpeed) {
  if (!_secondStepApplied) {
    std::cout << "[Sundaresan] Etapa: segundo degrau aplicado — cronometrando "
              << (kPercentT1 * 100) << "% e " << (kPercentT2 * 100) << "% do valor final\n";
    _secondStepApplied = true;
    _stepCount = 0;
    return;
  }

  _stepCount++;
  const double t = _stepCount * kSimulationDt;
  if (!_t1Set && filteredSpeed >= kPercentT1 * _finalValue) {
    _t1 = t;
    _t1Set = true;
    std::cout << "  " << (kPercentT1 * 100) << "% do valor final atingido em t1 = " << _t1 << " s\n";
  }
  if (!_t2Set && filteredSpeed >= kPercentT2 * _finalValue) {
    _t2 = t;
    _t2Set = true;
    std::cout << "  " << (kPercentT2 * 100) << "% do valor final atingido em t2 = " << _t2 << " s\n";
  }
  if (_t1Set && _t2Set) {
    finishPlantModel();
    _state = SundaresanState::Done;
  }
}

void Sundaresan::finishPlantModel() {
  _plantModel.theta = std::max(0.0, 1.3 * _t1 - 0.29 * _t2);
  _plantModel.tau = std::max(0.0, 0.67 * (_t2 - _t1));
  _plantModel.K = _finalValue / kIdentificationStep;
  std::cout << "Sundaresan-Krishnaswamy: t1 = " << _t1 << " s, t2 = " << _t2 << " s\n";
}
