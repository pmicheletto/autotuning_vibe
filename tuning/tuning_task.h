#ifndef TUNING_TUNING_TASK_H_
#define TUNING_TUNING_TASK_H_

#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "controllers/controller_type.h"
#include "models.h"
#include "oscillation_counter.h"
#include "settling_detector.h"
#include "system/system_interface.h"
#include "transfer_function/sundaresan.h"

// PID autotuning on an ISystem plant:
//   PLANT_IDENTIFICATION  Sundaresan-Krishnaswamy FOPDT identification (open loop)
//   CONTROLLER_TUNING     wait for rest, build Skogestad candidates (tau_c grid)
//   TEST_CONTROLLER       closed-loop step test of each candidate to steady state;
//                         a second round adds the two neighbours of the best one
//   COMPLETED / ERROR
// Drive it with stepSimulation() + update() once per simulation step.
enum class TuningState { PLANT_IDENTIFICATION, CONTROLLER_TUNING, TEST_CONTROLLER, COMPLETED, ERROR };

// Closed-loop test phases of one candidate.
enum class TestPhase {
  Prepare,        // open loop, wait for rest, then apply the PID and close the loop
  WaitTransient,  // reference step applied, waiting for the output to move
  WaitTouchRef,   // rising, waiting for the first entry into the settling band
  Analyze         // measuring until settled
};

struct CandidateResult {
  static constexpr double kDiscardedScore = -1e9;
  static constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

  double score{kDiscardedScore};  // -IAE minus penalties; kDiscardedScore if the test failed
  double iae{kNaN};
  double settlingTimeS{kNaN};
  double overshootPercent{0.0};
  int oscillations{0};
  double peakOutput{kNaN};
  double jerkAbsMax{kNaN};        // from the samples fed via noteJerkSample()

  bool discarded() const { return score <= kDiscardedScore; }
};

struct Candidate {
  double tauC{0.0};
  PIDParameters params;
  CandidateResult result;
};

class TuningTask {
 public:
  TuningTask(ControllerType controllerType, ISystem& system);

  void setSimulationStep(double dt) { _dt = dt; }
  double getSimulationStep() const { return _dt; }

  void stepSimulation();
  void update();

  TuningState getState() const { return _state; }
  bool isCompleted() const { return _state == TuningState::COMPLETED; }
  bool hasError() const { return _state == TuningState::ERROR; }

  // Results (throw unless completed).
  PIDParameters getResult() const;
  const Candidate& getBestCandidate() const;
  // All candidates in test order (phase 1 grid first, then the refinement pair).
  const std::vector<Candidate>& getCandidates() const { return _candidates; }

  // Identified FOPDT model and output noise (valid after identification).
  const PlantModel& getPlantModel() const { return _plantModel; }
  double getNoise() const { return _noise; }
  // Reference step height used by the closed-loop tests.
  static constexpr double kClosedLoopReference = 36.0;

  // Live-stream helpers: current candidate number (0 outside the tests), phase
  // name ("-" outside the tests) and max |jerk| seen for the current candidate.
  size_t currentCandidateNumber() const;
  std::string testPhaseName() const;
  double currentCandidateJerkAbsMax() const { return _currentJerkAbsMax; }
  // Feeds a jerk estimate while a candidate's step response is running.
  void noteJerkSample(std::optional<double> jerk);

  // Time budget (s) for one candidate's step response: 10 (tau_c + theta), clamped.
  static double candidateTestBudgetS(double tauC, double theta);

 private:
  void runIdentification();
  void runPostIdentificationWait();
  void buildCandidates();
  void addCandidate(double tauC);
  void appendRefinementCandidates();
  void runCandidateTest();
  void runPrepare(double output);
  void runWaitTransient(double output);
  void runWaitTouchRef(double output);
  void runAnalyze(double output);
  void startStepResponse();
  void accumulateStepResponse(double output);
  bool stepResponseBudgetExceeded() const;
  void finishCandidate(const CandidateResult& result);
  void failCandidate(const char* reason);
  size_t selectBest(size_t count) const;
  Candidate& current() { return _candidates[_currentIndex]; }

  // --- configuration -------------------------------------------------------
  static constexpr double kTestBudgetFactor = 10.0;
  static constexpr double kTestBudgetMinS = 60.0;
  static constexpr double kTestBudgetMaxS = 300.0;
  static constexpr double kSettleHoldS = 5.0;
  static constexpr double kSettlingBandFrac = 0.02;
  static constexpr double kSettlingBandNoiseMult = 5.0;
  static constexpr double kOvershootPenaltyThreshold = 1.05;  // > 5% overshoot -> kOvershootPenaltyScore
  static constexpr double kOvershootPenaltyScore = -1e6;
  static constexpr double kOscillationPenalty = 100.0;
  // Rest before each test / after identification: |out| <= max(abs, factor * noise).
  static constexpr double kRestAbsTol = 5e-5;
  static constexpr double kRestNoiseFactor = 2.0;
  static constexpr unsigned kPrepareRestSteps = 80;
  static constexpr unsigned kPrepareMaxSteps = 250000;
  static constexpr unsigned kPostIdentificationRestSteps = 7000;
  // Transient detected when the output moved by more than this from its level at the step.
  static constexpr double kTransientMoveFrac = 0.0004;  // of the reference
  static constexpr double kTransientMoveAbsMin = 0.008;
  // Phase 1 grid: kTcProbeMinS + i * kTcProbeStepS, i = 0..kTcProbeCount-1.
  static constexpr double kTcProbeMinS = 6.0;
  static constexpr double kTcProbeStepS = 3.0;
  static constexpr int kTcProbeCount = 3;
  // Phase 2: tau_c* (1 +/- kTcLateralFraction), skipped if within kTcNearRelativeTol of an existing one.
  static constexpr double kTcLateralFraction = 0.12;
  static constexpr double kTcNearRelativeTol = 0.04;

  // --- state -----------------------------------------------------------------
  ControllerType _controllerType;
  ISystem* _system;
  Sundaresan _sundaresan;
  double _dt{0.001};
  TuningState _state{TuningState::PLANT_IDENTIFICATION};
  PlantModel _plantModel;
  double _noise{0.0};
  unsigned _restStreak{0};
  unsigned _phaseSteps{0};

  std::vector<Candidate> _candidates;
  size_t _round1Count{0};
  size_t _currentIndex{0};
  size_t _bestIndex{0};
  TestPhase _phase{TestPhase::Prepare};

  // Current step response.
  std::unique_ptr<SettlingDetector> _settling;
  std::unique_ptr<OscillationCounter> _oscillations;
  unsigned _responseSteps{0};
  double _responseSumAbsError{0.0};
  double _maxOutput{0.0};
  double _transientStartOutput{0.0};
  double _currentJerkAbsMax{std::numeric_limits<double>::quiet_NaN()};
};

#endif  // TUNING_TUNING_TASK_H_
