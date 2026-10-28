#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "test/support/fake_plant.h"
#include "tuning/controllers/controller_type.h"
#include "tuning/parameters/skogestad_tuner.h"
#include "tuning/tuning_task.h"

namespace {

constexpr double kDt = 0.001;
constexpr double kReference = TuningTask::kClosedLoopReference;

// Runs the whole tuning (identification -> candidates -> selection).
void runTuning(TuningTask& task, double maxSimS = 1500.0) {
  task.setSimulationStep(kDt);
  for (int i = 0; i < static_cast<int>(maxSimS / kDt); ++i) {
    task.stepSimulation();
    task.update();
    if (task.isCompleted() || task.hasError()) break;
  }
}

}  // namespace

TEST(TuningTask, ResultsAreUnavailableBeforeCompletion) {
  FakePlant plant(40.0, 4.25, 0.5);
  TuningTask task(ControllerType::PID, plant);
  EXPECT_THROW(task.getResult(), std::runtime_error);
  EXPECT_EQ(task.currentCandidateNumber(), 0u);
  EXPECT_EQ(task.testPhaseName(), "-");
}

TEST(TuningTask, CandidateTestBudgetScalesWithTauCAndIsClamped) {
  EXPECT_DOUBLE_EQ(TuningTask::candidateTestBudgetS(6.0, 0.0), 60.0);      // floor
  EXPECT_DOUBLE_EQ(TuningTask::candidateTestBudgetS(13.44, 0.0), 134.4);   // 10 * tau_c
  EXPECT_DOUBLE_EQ(TuningTask::candidateTestBudgetS(12.0, 2.0), 140.0);    // 10 * (tau_c + theta)
  EXPECT_DOUBLE_EQ(TuningTask::candidateTestBudgetS(50.0, 10.0), 300.0);   // cap
  EXPECT_DOUBLE_EQ(TuningTask::candidateTestBudgetS(-1.0, -1.0), 60.0);    // garbage in -> floor
}

TEST(TuningTask, EveryCandidateIsTestedToSteadyStateAndScored) {
  const double K = 40.0, tau = 4.25, theta = 0.5;
  FakePlant plant(K, tau, theta);
  TuningTask task(ControllerType::PID, plant);

  runTuning(task);
  ASSERT_TRUE(task.isCompleted());
  ASSERT_FALSE(task.hasError());

  // Phase 1 probes tau_c = 6, 9, 12; phase 2 adds the two neighbours of the best.
  const auto& candidates = task.getCandidates();
  ASSERT_EQ(candidates.size(), 5u);
  EXPECT_DOUBLE_EQ(candidates[0].tauC, 6.0);
  EXPECT_DOUBLE_EQ(candidates[1].tauC, 9.0);
  EXPECT_DOUBLE_EQ(candidates[2].tauC, 12.0);
  for (const Candidate& c : candidates) {
    EXPECT_FALSE(c.result.discarded()) << "tau_c " << c.tauC << " was discarded";
    EXPECT_TRUE(std::isfinite(c.result.settlingTimeS));
  }
  // One FMU/plant reset per candidate test.
  EXPECT_EQ(plant.resets(), 5);
}

TEST(TuningTask, ChoosesTheFastestCandidateAndReportsItsMetrics) {
  // Without jerk samples the selection falls back to the score (-IAE, minus
  // penalties). On an ideal FOPDT the SIMC closed loop is ~first order with time
  // constant tau_c, so IAE ~ r (tau_c + theta) and the smallest tau_c wins:
  // 6 s in phase 1, then its lower neighbour 6 * 0.88 = 5.28 s in phase 2.
  const double K = 40.0, tau = 4.25, theta = 0.5;
  FakePlant plant(K, tau, theta);
  TuningTask task(ControllerType::PID, plant);
  runTuning(task);
  ASSERT_TRUE(task.isCompleted());

  const Candidate& best = task.getBestCandidate();
  const double tauC = best.tauC;
  EXPECT_NEAR(tauC, 5.28, 1e-9);

  // Gains are the SIMC formulas applied to the identified model.
  const PIDParameters expected = SkogestadTuner::calculatePID(task.getPlantModel(), tauC);
  const PIDParameters result = task.getResult();
  EXPECT_NEAR(result.Kp, expected.Kp, 1e-12);
  EXPECT_NEAR(result.Ti, expected.Ti, 1e-12);
  EXPECT_NEAR(result.Td, expected.Td, 1e-12);

  // Measured closed-loop performance of a ~first-order response with tau_c:
  // 2% settling ~ 4 tau_c + theta, no overshoot, no oscillation, IAE ~ r (tau_c + theta).
  EXPECT_NEAR(best.result.settlingTimeS, 4.0 * tauC + theta, 0.25 * (4.0 * tauC + theta));
  EXPECT_LT(best.result.overshootPercent, 5.0);
  EXPECT_EQ(best.result.oscillations, 0);
  EXPECT_NEAR(best.result.iae, kReference * (tauC + theta), 0.25 * kReference * (tauC + theta));
  EXPECT_NEAR(best.result.score, -best.result.iae, 1e-9);  // no penalties
}

TEST(TuningTask, TestsAreNotCutOffDuringTheRise) {
  // Regression for the 10 s cut-off: a slow loop (tau_c = 12 s needs ~50 s to
  // settle) must still be measured to steady state, not discarded mid-rise.
  FakePlant plant(40.0, 4.25, 0.5);
  TuningTask task(ControllerType::PID, plant);
  runTuning(task);
  ASSERT_TRUE(task.isCompleted());
  for (const Candidate& c : task.getCandidates()) {
    if (c.tauC == 12.0) {
      EXPECT_FALSE(c.result.discarded());
      // Score = -IAE for a clean response; IAE ~ r (tau_c + theta) = 36 * 12.5.
      EXPECT_NEAR(-c.result.score, 36.0 * 12.5, 0.25 * 36.0 * 12.5);
    }
  }
}

TEST(TuningTask, CandidateThatCannotReachTheReferenceIsDiscardedAfterItsBudget) {
  // Plant gain too small for a 0..1 pedal to ever reach 36? FakePlant is linear,
  // so instead cap its output below the reference.
  FakePlant plant(40.0, 4.25, 0.5);
  plant.setOutputLimit(30.0);  // steady state can never enter the band around 36
  TuningTask task(ControllerType::PID, plant);
  runTuning(task, 3000.0);
  ASSERT_TRUE(task.isCompleted());  // tuning still finishes, nothing hangs
  for (const Candidate& c : task.getCandidates()) {
    EXPECT_TRUE(c.result.discarded());
    EXPECT_TRUE(std::isnan(c.result.settlingTimeS));
  }
}
