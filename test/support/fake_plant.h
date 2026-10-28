#ifndef TEST_SUPPORT_FAKE_PLANT_H_
#define TEST_SUPPORT_FAKE_PLANT_H_

#include <cmath>
#include <deque>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>

#include "tuning/system/system_interface.h"

// Deterministic first-order-plus-dead-time plant behind the same variable
// interface the Modelica models expose (out, driver.step.height,
// driver.useManualControl, driver.pid.{k,Ti,Td,Nd}), with an ideal PID in the
// loop when manual control is off. Lets the tuning logic be exercised without
// an FMU, with analytically known answers.
//
//   y' = (K * u(t - theta) - y) / tau        (exact discretisation per step)
//   u  = step.height                           (manual / open loop)
//   u  = Kp * (e + (1/Ti) * integral(e) + Td * de/dt)   e = step.height - y (closed loop)
class FakePlant : public ISystem {
 public:
  FakePlant(double K, double tau, double theta) : _K(K), _tau(tau), _theta(theta) { reset(); }

  void reset() override {
    _vars.clear();
    _vars["out"] = 0.0;
    _vars["driver.step.height"] = 0.0;
    _vars["driver.step.y"] = 0.0;
    _vars["driver.useManualControl"] = 1.0;
    _vars["driver.pid.k"] = 1.0;
    _vars["driver.pid.Ti"] = 0.5;
    _vars["driver.pid.Td"] = 0.0;
    _vars["driver.pid.Nd"] = 10.0;
    _y = 0.0;
    _integral = 0.0;
    _prevError = 0.0;
    _derivative = 0.0;
    _delay.clear();
    _time = 0.0;
    ++_resets;
  }

  void advanceSimulation(double dt) override {
    const double ref = _vars["driver.step.height"];
    _vars["driver.step.y"] = ref;
    double u = ref;
    if (_vars["driver.useManualControl"] == 0.0) {
      const double kp = _vars["driver.pid.k"], ti = _vars["driver.pid.Ti"], td = _vars["driver.pid.Td"];
      const double nd = _vars["driver.pid.Nd"];
      const double e = ref - _y;
      _integral += e * dt;
      // Derivative with first-order filter, time constant Td/Nd.
      const double rawDerivative = (e - _prevError) / dt;
      const double tf = (td > 0.0) ? td / nd : 0.0;
      _derivative = (tf > 0.0) ? _derivative + (dt / (tf + dt)) * (rawDerivative - _derivative) : 0.0;
      _prevError = e;
      u = kp * (e + ((ti > 0.0) ? _integral / ti : 0.0) + td * _derivative);
    }

    // Dead time as a delay line of exactly theta/dt samples (input 0 before).
    const std::size_t delaySteps = static_cast<std::size_t>(std::llround(_theta / dt));
    _delay.push_back(u);
    double uDelayed = 0.0;
    if (_delay.size() > delaySteps) {
      uDelayed = _delay.front();
      _delay.pop_front();
    }

    const double a = std::exp(-dt / _tau);
    _y = a * _y + (1.0 - a) * _K * uDelayed;
    if (_y > _outputLimit) _y = _outputLimit;
    _vars["out"] = _y;
    _time += dt;
  }

  double readValue(const std::string& name) override {
    const auto it = _vars.find(name);
    if (it == _vars.end()) {
      throw std::runtime_error("FakePlant: no such variable '" + name + "'");
    }
    return it->second;
  }

  void setVariable(const std::string& name, double value) override { _vars[name] = value; }

  // Saturates the output (e.g. a plant that cannot reach the reference).
  void setOutputLimit(double limit) { _outputLimit = limit; }

  double time() const { return _time; }
  int resets() const { return _resets; }

 private:
  double _K, _tau, _theta;
  double _outputLimit{std::numeric_limits<double>::infinity()};
  std::map<std::string, double> _vars;
  double _y{0.0};
  double _integral{0.0};
  double _prevError{0.0};
  double _derivative{0.0};
  std::deque<double> _delay;
  double _time{0.0};
  int _resets{-1};
};

#endif  // TEST_SUPPORT_FAKE_PLANT_H_
