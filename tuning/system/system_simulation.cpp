#include <map>
#include <stdexcept>

#include "simulation/ModelicaWrapper.h"
#include "system_interface.h"

namespace {

constexpr double kStartTimeS = 0.0;

}  // namespace

// FMI co-simulation plant.
//
// Note on OpenModelica FMUs: the tuning changes *parameters* (PID gains,
// manual/automatic switch, reference step height) after initialisation, which
// FMI 2.0 does not formally allow. OpenModelica-generated FMUs tolerate it, but
// only apply such writes reliably when the values are sent again right before
// each fmi2DoStep and followed by an fmi2GetReal, which triggers the FMU's lazy
// re-evaluation of the model equations. Without that, the model's discrete
// logic silently falls back to start values (e.g. the driver pedal stays at 0),
// the dynamics depend on the caller's read pattern, and the non-linear solver
// eventually fails. `advanceSimulation` therefore re-sends every value written
// through `setVariable` and performs one read before stepping.
class SimulationSystem : public ISystem {
 public:
  explicit SimulationSystem(const std::string& fmuPath)
      : _fmuPath(fmuPath), _wrapper(std::make_unique<ModelicaWrapper>(fmuPath)) {
    _wrapper->init(kStartTimeS);
  }

  void reset() override {
    _wrapper = std::make_unique<ModelicaWrapper>(_fmuPath);
    _wrapper->init(kStartTimeS);
    _writtenValues.clear();  // fresh instance starts from the model defaults
  }

  void advanceSimulation(double dt) override {
    commitWrites();
    _wrapper->stepFMU(dt);
  }

  double readValue(const std::string& varName) override {
    return _wrapper->getReal(varName);
  }

  void setVariable(const std::string& varName, double value) override {
    _wrapper->setReal(varName, value);
    _writtenValues[varName] = value;
  }

 private:
  // Re-sends all values written so far and forces the FMU to re-evaluate (see
  // the class comment).
  void commitWrites() {
    if (_writtenValues.empty()) {
      return;
    }
    for (const auto& [name, value] : _writtenValues) {
      _wrapper->setReal(name, value);
    }
    (void)_wrapper->getReal(_writtenValues.begin()->first);
  }

  std::string _fmuPath;
  std::unique_ptr<ModelicaWrapper> _wrapper;
  std::map<std::string, double> _writtenValues;
};

std::unique_ptr<ISystem> createSystem(const std::string& fmuPath) {
  if (fmuPath.empty()) {
    throw std::invalid_argument("FMU path is required for simulation");
  }
  return std::make_unique<SimulationSystem>(fmuPath);
}
