#include "tuning_task.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

#include "loop_control.h"
#include "parameters/parameter_validator.h"
#include "parameters/skogestad_tuner.h"

namespace {

constexpr const char* kOutputVar = "out";
constexpr const char* kStepVar = "driver.step.height";

std::ostream& operator<<(std::ostream& os, const CandidateResult& r) {
  os << "tempo_acomodacao_s=" << r.settlingTimeS << ", pico=" << r.peakOutput << ", sobressinal="
     << std::fixed << std::setprecision(2) << r.overshootPercent << "%" << std::defaultfloat
     << ", oscilacoes=" << r.oscillations << ", IAE=" << r.iae << ", jerk_abs_max=" << r.jerkAbsMax
     << ", score=" << r.score;
  return os;
}

}  // namespace

TuningTask::TuningTask(ControllerType controllerType, ISystem& system)
    : _controllerType(controllerType), _system(&system), _sundaresan(system) {}

// ---------------------------------------------------------------------------
// Public helpers
// ---------------------------------------------------------------------------

PIDParameters TuningTask::getResult() const { return getBestCandidate().params; }

const Candidate& TuningTask::getBestCandidate() const {
  if (_state != TuningState::COMPLETED) {
    throw std::runtime_error("TuningTask: tuning is not complete");
  }
  return _candidates[_bestIndex];
}

size_t TuningTask::currentCandidateNumber() const {
  return _state == TuningState::TEST_CONTROLLER ? _currentIndex + 1 : 0;
}

std::string TuningTask::testPhaseName() const {
  if (_state != TuningState::TEST_CONTROLLER) {
    return "-";
  }
  switch (_phase) {
    case TestPhase::Prepare: return "Prepare";
    case TestPhase::WaitTransient: return "WaitTransient";
    case TestPhase::WaitTouchRef: return "WaitTouchRef";
    case TestPhase::Analyze: return "Analyze";
  }
  return "?";
}

void TuningTask::noteJerkSample(std::optional<double> jerk) {
  const bool responding = _state == TuningState::TEST_CONTROLLER &&
                          (_phase == TestPhase::WaitTouchRef || _phase == TestPhase::Analyze);
  if (!responding || !jerk || !std::isfinite(*jerk)) {
    return;
  }
  const double a = std::fabs(*jerk);
  _currentJerkAbsMax = std::isnan(_currentJerkAbsMax) ? a : std::max(_currentJerkAbsMax, a);
}

double TuningTask::candidateTestBudgetS(double tauC, double theta) {
  const double raw = kTestBudgetFactor * (std::max(tauC, 0.0) + std::max(theta, 0.0));
  return std::clamp(raw, kTestBudgetMinS, kTestBudgetMaxS);
}

// ---------------------------------------------------------------------------
// Main state machine
// ---------------------------------------------------------------------------

void TuningTask::stepSimulation() { _system->advanceSimulation(_dt); }

void TuningTask::update() {
  switch (_state) {
    case TuningState::PLANT_IDENTIFICATION: runIdentification(); break;
    case TuningState::CONTROLLER_TUNING: runPostIdentificationWait(); break;
    case TuningState::TEST_CONTROLLER: runCandidateTest(); break;
    case TuningState::COMPLETED:
    case TuningState::ERROR: break;
  }
}

void TuningTask::runIdentification() {
  loop_control::openLoop(*_system);
  _sundaresan.Execute();
  if (!_sundaresan.IsFinished()) {
    return;
  }
  if (_sundaresan.GetState() != SundaresanState::Done) {
    _state = TuningState::ERROR;
    return;
  }
  _plantModel = _sundaresan.GetPlantModel();
  _noise = _sundaresan.GetNoise();
  std::cout << "Função de transferência encontrada (Sundaresan-Krishnaswamy):\n"
            << "  K = " << _plantModel.K << ", tau = " << _plantModel.tau
            << " s, theta = " << _plantModel.theta << " s\n"
            << "[Tuning] Pós-identificação: aguardando speed ~= 0 por " << kPostIdentificationRestSteps
            << " amostras antes dos testes de ganhos.\n";
  _restStreak = 0;
  _phaseSteps = 0;
  _state = TuningState::CONTROLLER_TUNING;
}

void TuningTask::runPostIdentificationWait() {
  loop_control::openLoop(*_system);
  _system->setVariable(kStepVar, 0.0);
  ++_phaseSteps;
  const double restTol = std::max(kRestAbsTol, kRestNoiseFactor * _noise);
  _restStreak = (std::fabs(_system->readValue(kOutputVar)) <= restTol) ? _restStreak + 1 : 0;
  if (_restStreak < kPostIdentificationRestSteps) {
    return;
  }
  std::cout << "[Tuning] Pós-identificação concluída (" << _phaseSteps * _dt
            << " s) — iniciando testes de ganhos.\n";
  buildCandidates();
}

// ---------------------------------------------------------------------------
// Candidates
// ---------------------------------------------------------------------------

void TuningTask::addCandidate(double tauC) {
  const PIDParameters params = (_controllerType == ControllerType::PI)
                                   ? SkogestadTuner::calculatePI(_plantModel, tauC)
                                   : SkogestadTuner::calculatePID(_plantModel, tauC);
  if (!ParameterValidator::validate(params)) {
    return;
  }
  _candidates.push_back(Candidate{tauC, params, {}});
  std::cout << "  [" << _candidates.size() << "] τ_c = " << tauC << ", Kp = " << params.Kp
            << ", Ti = " << params.Ti << ", Td = " << params.Td << "\n";
}

void TuningTask::buildCandidates() {
  std::cout << "Fase 1 — " << kTcProbeCount << " ganhos (τ_c de " << kTcProbeMinS << " s, passo "
            << kTcProbeStepS << " s):\n";
  for (int i = 0; i < kTcProbeCount; ++i) {
    addCandidate(kTcProbeMinS + i * kTcProbeStepS);
  }
  if (_candidates.empty()) {
    _state = TuningState::ERROR;
    return;
  }
  _round1Count = _candidates.size();
  _currentIndex = 0;
  _phase = TestPhase::Prepare;
  _phaseSteps = 0;
  _state = TuningState::TEST_CONTROLLER;
}

void TuningTask::appendRefinementCandidates() {
  const double tcStar = _candidates[selectBest(_round1Count)].tauC;
  std::cout << "Fase 2 — refinamento em torno de τ_c = " << tcStar << " s:\n";
  for (double tc : {tcStar * (1.0 - kTcLateralFraction), tcStar * (1.0 + kTcLateralFraction)}) {
    const bool nearExisting = std::any_of(_candidates.begin(), _candidates.end(), [&](const Candidate& c) {
      return std::fabs(c.tauC - tc) <= kTcNearRelativeTol * std::max(std::fabs(c.tauC), std::fabs(tc));
    });
    if (!nearExisting) {
      addCandidate(std::max(tc, 1e-12));
    }
  }
}

// Lowest max |jerk| wins; candidates without a jerk measurement rank below
// those with one and are compared by score, which also breaks jerk ties.
size_t TuningTask::selectBest(size_t count) const {
  const auto better = [](const CandidateResult& a, const CandidateResult& b) {
    const bool ja = std::isfinite(a.jerkAbsMax), jb = std::isfinite(b.jerkAbsMax);
    if (ja != jb) {
      return ja;
    }
    if (ja && std::fabs(a.jerkAbsMax - b.jerkAbsMax) > 1e-12) {
      return a.jerkAbsMax < b.jerkAbsMax;
    }
    return a.score > b.score;
  };
  size_t best = 0;
  for (size_t i = 1; i < count; ++i) {
    if (better(_candidates[i].result, _candidates[best].result)) {
      best = i;
    }
  }
  return best;
}

// ---------------------------------------------------------------------------
// Closed-loop step test of the current candidate
// ---------------------------------------------------------------------------

void TuningTask::runCandidateTest() {
  const double output = _system->readValue(kOutputVar);
  switch (_phase) {
    case TestPhase::Prepare: runPrepare(output); break;
    case TestPhase::WaitTransient: runWaitTransient(output); break;
    case TestPhase::WaitTouchRef: runWaitTouchRef(output); break;
    case TestPhase::Analyze: runAnalyze(output); break;
  }
}

void TuningTask::runPrepare(double output) {
  if (_phaseSteps == 0) {
    _system->reset();  // every candidate starts from the same plant state
    _restStreak = 0;
  }
  loop_control::openLoop(*_system);
  _system->setVariable(kStepVar, 0.0);
  ++_phaseSteps;

  const double restTol = std::max(kRestAbsTol, kRestNoiseFactor * _noise);
  _restStreak = (std::fabs(output) <= restTol) ? _restStreak + 1 : 0;
  const bool rested = _restStreak >= kPrepareRestSteps;
  if (!rested && _phaseSteps < kPrepareMaxSteps) {
    return;
  }
  if (!rested) {
    std::cout << "Aviso: saida nao estabilizou perto de 0 antes do candidato " << (_currentIndex + 1)
              << " (|out|=" << output << ") — continua.\n";
  }

  const Candidate& c = current();
  loop_control::applyPID(*_system, c.params.Kp, c.params.Ti, c.params.Td);
  loop_control::closeLoop(*_system);
  std::cout << "[Tuning] Início candidato " << (_currentIndex + 1) << "/" << _candidates.size()
            << " (malha fechada): tau_c=" << c.tauC << ", Kp=" << c.params.Kp << ", Ti=" << c.params.Ti
            << ", Td=" << c.params.Td << ", ref=" << kClosedLoopReference << "\n";
  startStepResponse();
  _phase = TestPhase::WaitTransient;
  _phaseSteps = 0;
}

void TuningTask::runWaitTransient(double output) {
  _system->setVariable(kStepVar, kClosedLoopReference);
  if (_phaseSteps++ == 0) {
    _transientStartOutput = output;
  }
  accumulateStepResponse(output);
  const double moveTol = std::max(kTransientMoveFrac * kClosedLoopReference, kTransientMoveAbsMin);
  if (std::fabs(output - _transientStartOutput) > moveTol) {
    _currentJerkAbsMax = CandidateResult::kNaN;  // measure jerk from here on
    _phase = TestPhase::WaitTouchRef;
  } else if (stepResponseBudgetExceeded()) {
    failCandidate("nao respondeu ao degrau");
  }
}

void TuningTask::runWaitTouchRef(double output) {
  _system->setVariable(kStepVar, kClosedLoopReference);
  accumulateStepResponse(output);
  if (std::fabs(output - kClosedLoopReference) <= _settling->band()) {
    _oscillations->reset();
    _phase = TestPhase::Analyze;
  } else if (stepResponseBudgetExceeded()) {
    failCandidate("nao atingiu a referencia");
  }
}

void TuningTask::runAnalyze(double output) {
  _system->setVariable(kStepVar, kClosedLoopReference);
  accumulateStepResponse(output);
  _oscillations->update(output - kClosedLoopReference);
  if (_settling->settled()) {
    CandidateResult r;
    r.iae = _responseSumAbsError;
    r.settlingTimeS = _settling->settlingTimeS();
    r.peakOutput = _maxOutput;
    r.overshootPercent = std::max(0.0, 100.0 * (_maxOutput - kClosedLoopReference) / kClosedLoopReference);
    r.oscillations = _oscillations->count();
    r.score = (_maxOutput > kClosedLoopReference * kOvershootPenaltyThreshold)
                  ? kOvershootPenaltyScore
                  : -r.iae - kOscillationPenalty * r.oscillations;
    finishCandidate(r);
  } else if (stepResponseBudgetExceeded()) {
    failCandidate("nao estabilizou na referencia");
  }
}

void TuningTask::startStepResponse() {
  SettlingDetector::Config cfg;
  cfg.dt = _dt;
  cfg.reference = kClosedLoopReference;
  cfg.holdS = kSettleHoldS;
  cfg.bandFrac = kSettlingBandFrac;
  cfg.noiseMult = kSettlingBandNoiseMult;
  cfg.noise = _noise;
  _settling = std::make_unique<SettlingDetector>(cfg);
  _oscillations = std::make_unique<OscillationCounter>(_settling->band(), OscillationCounter::minOutsideSteps(_dt));
  _responseSteps = 0;
  _responseSumAbsError = 0.0;
  _maxOutput = 0.0;
  _currentJerkAbsMax = CandidateResult::kNaN;
}

void TuningTask::accumulateStepResponse(double output) {
  ++_responseSteps;
  _responseSumAbsError += std::fabs(output - kClosedLoopReference) * _dt;
  _maxOutput = std::max(_maxOutput, output);
  _settling->update(output);
}

bool TuningTask::stepResponseBudgetExceeded() const {
  return _responseSteps * _dt > candidateTestBudgetS(_candidates[_currentIndex].tauC, _plantModel.theta);
}

void TuningTask::failCandidate(const char* reason) {
  std::cout << "[Tuning] Candidato " << (_currentIndex + 1) << " descartado: " << reason
            << " dentro do orcamento de " << candidateTestBudgetS(current().tauC, _plantModel.theta)
            << " s (saida " << _system->readValue(kOutputVar) << ", ref " << kClosedLoopReference << ").\n";
  finishCandidate(CandidateResult{});
}

void TuningTask::finishCandidate(const CandidateResult& result) {
  Candidate& c = current();
  c.result = result;
  c.result.jerkAbsMax = _currentJerkAbsMax;
  std::cout << "[Tuning] Teste candidato " << (_currentIndex + 1) << "/" << _candidates.size()
            << ": tau_c=" << c.tauC << ", " << c.result << "\n";

  ++_currentIndex;
  if (_currentIndex == _round1Count && _candidates.size() == _round1Count) {
    appendRefinementCandidates();
  }
  if (_currentIndex < _candidates.size()) {
    _phase = TestPhase::Prepare;
    _phaseSteps = 0;
    return;
  }
  _bestIndex = selectBest(_candidates.size());
  _state = TuningState::COMPLETED;
  const Candidate& best = _candidates[_bestIndex];
  std::cout << "[Tuning] Ganhos escolhidos: Kp=" << best.params.Kp << ", Ti=" << best.params.Ti
            << ", Td=" << best.params.Td << ", tau_c=" << best.tauC << " — " << best.result << "\n";
}
