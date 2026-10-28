#ifndef TUNING_MODELS_H_
#define TUNING_MODELS_H_

struct PIDParameters {
  double Kp{0.0};
  double Ti{0.0};
  double Td{0.0};
};

// First-order-plus-dead-time plant: K e^(-theta s) / (tau s + 1).
struct PlantModel {
  double K{0.0};
  double tau{0.0};
  double theta{0.0};
};

#endif  // TUNING_MODELS_H_
