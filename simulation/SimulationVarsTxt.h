#ifndef SIMULATION_SIMULATION_VARS_TXT_H_
#define SIMULATION_SIMULATION_VARS_TXT_H_

#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>

// CSV trace of the streamed samples (same columns as the UDP messages).
class SimulationVarsTxt {
 public:
  explicit SimulationVarsTxt(const std::string& path) : _out(path, std::ios::out | std::ios::trunc) {
    if (!_out) {
      throw std::runtime_error("SimulationVarsTxt: could not open " + path);
    }
    _out << "time,step,speed,jerk,speed_filt,jerk_max_current_gain_test,test_phase\n";
  }

  void append(double time, double step, double speed, std::optional<double> jerk,
              double speed_filtered, double jerk_max_current_gain_test,
              const std::string& test_phase) {
    _out << time << ',' << step << ',' << speed << ',';
    if (jerk.has_value()) {
      _out << *jerk;
    } else {
      _out << "nan";
    }
    _out << ',' << speed_filtered << ',' << jerk_max_current_gain_test << ',' << test_phase
         << '\n';
    _out.flush();  // rows are sparse (one per 100 steps); keep the trace on disk at all times
  }

 private:
  std::ofstream _out;
};

#endif  // SIMULATION_SIMULATION_VARS_TXT_H_
