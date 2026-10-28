#include "skogestad_tuner.h"

#include <algorithm>

namespace {

constexpr double kIntegralTimeTcFactor = 4.0;

}  // namespace

PIDParameters SkogestadTuner::calculatePI(const PlantModel& model, double tc) {
  PIDParameters params;
  const double halfTheta = 0.5 * model.theta;
  const double den = tc + halfTheta;
  if (model.K == 0.0 || den <= 0.0) {
    return params;
  }
  params.Kp = (model.tau + halfTheta) / (model.K * den);
  params.Ti = std::min(model.tau + halfTheta, kIntegralTimeTcFactor * den);
  params.Td = 0.0;
  return params;
}

PIDParameters SkogestadTuner::calculatePID(const PlantModel& model, double tc) {
  PIDParameters params = calculatePI(model, tc);
  const double twoTauPlusTheta = 2.0 * model.tau + model.theta;
  if (params.Kp != 0.0 && twoTauPlusTheta > 0.0) {
    params.Td = (model.tau * model.theta) / twoTauPlusTheta;
  }
  return params;
}
