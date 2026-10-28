#ifndef TUNING_SYSTEM_SYSTEM_INTERFACE_H_
#define TUNING_SYSTEM_SYSTEM_INTERFACE_H_

#include <memory>
#include <string>

// Plant abstraction used by the tuning algorithms. The only implementation is
// the FMI co-simulation system (see system_simulation.cpp).
class ISystem {
 public:
  virtual ~ISystem() = default;

  virtual void reset() = 0;

  // Advances the simulation by `dt` seconds.
  virtual void advanceSimulation(double dt) = 0;

  virtual double readValue(const std::string& varName) = 0;

  virtual void setVariable(const std::string& varName, double value) = 0;
};

// Creates an FMI co-simulation system from an FMU file.
// Throws std::invalid_argument if `fmuPath` is empty.
std::unique_ptr<ISystem> createSystem(const std::string& fmuPath);

#endif  // TUNING_SYSTEM_SYSTEM_INTERFACE_H_
