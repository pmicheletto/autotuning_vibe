// PID autotuning on an FMU plant: identification -> Skogestad candidates ->
// closed-loop tests -> validation of the chosen gains -> report. Samples are
// streamed over UDP to the live plot (tools/live_plot.py) and to a CSV trace.

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include "simulation/SimulationVarsTxt.h"
#include "simulation/UDPSender.h"
#include "simulation/live_plot.h"
#include "statistics/JerkEstimator.h"
#include "tuning/control_report.h"
#include "tuning/control_validation.h"
#include "tuning/controllers/controller_type.h"
#include "tuning/system/system_interface.h"
#include "tuning/tuning_task.h"

namespace fs = std::filesystem;

namespace {

constexpr const char* kDefaultFmuName = "variantC.fmu";
// Paths relative to the executable (build/sim); the source tree is the fallback.
constexpr const char* kModelsDirFromExe = "../simulation/modelica_models";
constexpr const char* kLivePlotScriptFromExe = "../tools/live_plot.py";

constexpr double kSimulationStepS = 0.001;
constexpr int kMaxSimulationSteps = 1'000'000;
constexpr int kProgressLogEveryNSteps = 5000;

// Streaming: one UDP/CSV sample (and one jerk-estimator sample) every N steps.
constexpr const char* kUdpHost = "127.0.0.1";
constexpr int kUdpPort = 9870;
constexpr int kStreamEveryNSteps = 100;
constexpr const char* kTraceFileName = "sim_trace.txt";
constexpr const char* kLivePlotLogName = "live_plot.log";
constexpr int kLivePlotHttpPort = 8088;

constexpr unsigned kJerkSpeedMeanWindow = 20;
constexpr unsigned kJerkMeanWindow = 20;
constexpr double kJerkMinSampleIntervalS = 0.05;

// Final validation of the chosen gains.
constexpr double kValidationMaxDurationS = 120.0;
constexpr double kValidationSettleHoldS = 5.0;

struct Options {
  std::string fmuArg;
  bool livePlot{true};
  bool keepPlot{false};
  int httpPort{kLivePlotHttpPort};
};

void printUsage(const char* argv0) {
  std::cout << "Usage: " << argv0 << " [options] [FMU|model-name]\n"
            << "  --no-plot           do not start the live plot (tools/live_plot.py)\n"
            << "  --plot-port PORT    HTTP port for the live plot (0 = auto, default "
            << kLivePlotHttpPort << ")\n"
            << "  --keep-plot         after the report, keep running (and the live plot "
               "open) until Enter is pressed\n"
            << "  -h, --help          show this help\n"
            << "Environment: AUTOTUNING_NO_PLOT=1 disables the live plot.\n";
}

// std::nullopt means "usage printed, exit".
std::optional<Options> parseOptions(int argc, char** argv) {
  Options options;
  if (const char* env = std::getenv("AUTOTUNING_NO_PLOT"); env && std::string(env) != "0") {
    options.livePlot = false;
  }
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--no-plot") {
      options.livePlot = false;
    } else if (arg == "--keep-plot") {
      options.keepPlot = true;
    } else if (arg == "--plot-port" && i + 1 < argc) {
      options.httpPort = std::stoi(argv[++i]);
    } else if (arg == "-h" || arg == "--help") {
      printUsage(argv[0]);
      return std::nullopt;
    } else if (arg.size() > 1 && arg[0] == '-') {
      throw std::runtime_error("unknown option: " + arg);
    } else if (options.fmuArg.empty()) {
      options.fmuArg = arg;
    } else {
      throw std::runtime_error("unexpected argument: " + arg);
    }
  }
  return options;
}

fs::path executableDirectory(const char* argv0) {
  std::error_code ec;
  fs::path exe = fs::read_symlink("/proc/self/exe", ec);
  if (ec || exe.empty()) {
    exe = fs::absolute(argv0, ec);
  }
  return exe.parent_path();
}

// `arg`: empty (default model), a path, or a model name (with or without .fmu)
// looked up in the models directory.
fs::path resolveFmuPath(const std::string& arg, const fs::path& exeDir) {
  std::vector<fs::path> modelDirs{exeDir / kModelsDirFromExe};
#ifdef AUTOTUNING_MODELS_DIR
  modelDirs.emplace_back(AUTOTUNING_MODELS_DIR);
#endif
  std::vector<fs::path> candidates;
  if (arg.empty()) {
    for (const fs::path& dir : modelDirs) candidates.push_back(dir / kDefaultFmuName);
  } else {
    candidates.emplace_back(arg);
    for (const fs::path& dir : modelDirs) {
      candidates.push_back(dir / arg);
      candidates.push_back(dir / (arg + ".fmu"));
    }
  }
  for (const fs::path& c : candidates) {
    std::error_code ec;
    if (fs::is_regular_file(c, ec)) {
      return fs::weakly_canonical(c, ec);
    }
  }
  std::string msg = "FMU not found. Tried:";
  for (const fs::path& c : candidates) msg += "\n  " + c.string();
  throw std::runtime_error(msg);
}

// Starts the live plot unless disabled; never fatal.
std::unique_ptr<LivePlot> maybeStartLivePlot(const Options& options, const fs::path& exeDir,
                                             const fs::path& fmuPath) {
  if (!options.livePlot) {
    std::cout << "Live plot disabled; UDP stream still available on " << kUdpHost << ":" << kUdpPort
              << " (start tools/live_plot.py manually to view it).\n";
    return nullptr;
  }
  LivePlotOptions plot;
  plot.scriptPath = (exeDir / kLivePlotScriptFromExe).lexically_normal().string();
  std::error_code ec;
  if (!fs::is_regular_file(plot.scriptPath, ec)) {
#ifdef AUTOTUNING_TOOLS_DIR
    plot.scriptPath = (fs::path(AUTOTUNING_TOOLS_DIR) / "live_plot.py").string();
#endif
  }
  plot.udpHost = kUdpHost;
  plot.udpPort = kUdpPort;
  plot.httpPort = options.httpPort;
  plot.logPath = fs::absolute(kLivePlotLogName).string();
  plot.title = "PID autotuning - " + fmuPath.filename().string();
  plot.openBrowser = std::getenv("DISPLAY") || std::getenv("WAYLAND_DISPLAY");

  std::string error;
  auto process = startLivePlot(plot, &error);
  if (!process) {
    std::cerr << "Warning: live plot not started (" << error << "). Continuing without it.\n";
    return nullptr;
  }
  std::cout << "Live plot: " << process->url() << " (closes with this program; log "
            << process->logPath() << ")\n";
  if (!plot.openBrowser) {
    std::cout << "  No graphical session detected; open the URL manually.\n";
  }
  return process;
}

std::string phaseNameForTiming(TuningState state) {
  switch (state) {
    case TuningState::PLANT_IDENTIFICATION: return "Plant identification";
    case TuningState::CONTROLLER_TUNING: return "Post-identification settle";
    case TuningState::TEST_CONTROLLER: return "Gain candidate tests";
    case TuningState::COMPLETED: return "Completed";
    case TuningState::ERROR: return "Error";
  }
  return "?";
}

// JSON numbers must be finite; NaN/Inf become null.
std::string jsonNumber(double value) {
  return std::isfinite(value) ? std::to_string(value) : "null";
}

std::string jsonNumber(std::optional<double> value) {
  return value ? jsonNumber(*value) : "null";
}

// One streamed sample: CSV row + UDP datagram with the same fields.
class SampleStream {
 public:
  SampleStream(const std::string& tracePath) : _csv(tracePath), _udp(kUdpHost, kUdpPort) {}

  void emit(double t, double step, double speed, std::optional<double> jerk, double speedFilt,
            double jerkMax, const std::string& phase) {
    _csv.append(t, step, speed, jerk, speedFilt, jerkMax, phase);
    _udp.send("{\"time\": " + jsonNumber(t) + ", \"step\": " + jsonNumber(step) +
              ", \"speed\": " + jsonNumber(speed) + ", \"jerk\": " + jsonNumber(jerk) +
              ", \"speed_filt\": " + jsonNumber(speedFilt) +
              ", \"jerk_max_current_gain_test\": " + jsonNumber(jerkMax) +
              ", \"test_phase\": \"" + phase + "\"}");
  }

 private:
  SimulationVarsTxt _csv;
  UDPSender _udp;
};

}  // namespace

int main(int argc, char** argv) {
  try {
    const auto options = parseOptions(argc, argv);
    if (!options) {
      return 0;
    }
    const fs::path exeDir = executableDirectory(argv[0]);
    const fs::path fmuPath = resolveFmuPath(options->fmuArg, exeDir);
    const fs::path tracePath = fs::absolute(kTraceFileName);
    const auto livePlot = maybeStartLivePlot(*options, exeDir, fmuPath);

    auto system = createSystem(fmuPath.string());
    TuningTask task(ControllerType::PID, *system);
    task.setSimulationStep(kSimulationStepS);

    SampleStream stream(tracePath.string());
    JerkEstimator jerk(kJerkSpeedMeanWindow, kJerkMinSampleIntervalS, kJerkMeanWindow);
    PhaseTimer timer(kSimulationStepS);

    std::cout << "Tuning (Sundaresan ID + Skogestad PID). FMU: " << fmuPath.string() << "\n"
              << "UDP " << kUdpHost << ":" << kUdpPort << ", trace " << tracePath.string()
              << ", max " << kMaxSimulationSteps << " steps.\n";

    // ------------------------------------------------------------------
    // Tuning: identification -> candidates -> closed-loop tests
    // ------------------------------------------------------------------
    int tuningSteps = 0;
    size_t jerkCandidate = 0;  // candidate whose test the jerk estimator is tracking
    for (int i = 1; i <= kMaxSimulationSteps; ++i) {
      timer.step(phaseNameForTiming(task.getState()));
      task.stepSimulation();
      task.update();
      tuningSteps = i;

      // Jerk estimation restarts for each candidate's step response.
      const std::string phase = task.testPhaseName();
      const size_t candidate = task.currentCandidateNumber();
      if (phase == "Prepare" && candidate != jerkCandidate) {
        jerk.Clear();
        jerkCandidate = candidate;
      }

      if (i % kStreamEveryNSteps == 0) {
        const double t = i * kSimulationStepS;
        const double speed = system->readValue("out");
        std::optional<double> jerkSample;
        double speedFilt = speed;
        if (phase != "Prepare") {
          jerkSample = jerk.AddSpeed(speed, t);
          speedFilt = jerk.GetLastFilteredSpeed();
        }
        task.noteJerkSample(jerkSample);
        stream.emit(t, system->readValue("driver.step.y"), speed, jerkSample, speedFilt,
                    task.currentCandidateJerkAbsMax(), phase);
      }

      if (task.isCompleted() || task.hasError()) {
        std::cout << "Done: " << i << " steps.\n";
        break;
      }
      if (i % kProgressLogEveryNSteps == 0) {
        std::cout << "  " << i << " / " << kMaxSimulationSteps << " steps (" << i * kSimulationStepS
                  << " s)\n";
      }
    }

    if (task.hasError()) {
      std::cout << "Tuning failed (invalid identification or parameters).\n";
      return 1;
    }
    if (!task.isCompleted()) {
      std::cout << "Timeout: maximum steps reached.\n";
      return 1;
    }
    const Candidate& best = task.getBestCandidate();
    std::cout << "Tuning finished. PID parameters:\n  Kp = " << best.params.Kp
              << ", Ti = " << best.params.Ti << ", Td = " << best.params.Td << "\n";

    // ------------------------------------------------------------------
    // Validation: fresh closed-loop step test with the chosen gains
    // ------------------------------------------------------------------
    ClosedLoopStepTestConfig cfg;
    cfg.dt = kSimulationStepS;
    cfg.reference = TuningTask::kClosedLoopReference;
    cfg.maxDurationS = kValidationMaxDurationS;
    cfg.settleHoldS = kValidationSettleHoldS;
    cfg.noise = task.getNoise();
    cfg.jerkEveryNSteps = kStreamEveryNSteps;
    cfg.jerkSpeedMeanWindow = kJerkSpeedMeanWindow;
    cfg.jerkMinSampleIntervalS = kJerkMinSampleIntervalS;
    cfg.jerkMeanWindow = kJerkMeanWindow;

    std::cout << "[Validation] Closed-loop step 0 -> " << cfg.reference
              << " with the tuned PID (up to " << cfg.maxDurationS << " s)...\n";
    const double timeOffset = tuningSteps * kSimulationStepS;  // keep the trace monotonic
    const auto validationStart = std::chrono::steady_clock::now();
    const StepResponseMetrics metrics =
        runClosedLoopStepTest(*system, best.params, cfg, [&](const ClosedLoopSample& s) {
          if (s.index % kStreamEveryNSteps == 0) {
            stream.emit(timeOffset + s.timeS, s.stepHeight, s.output, s.jerk, s.output, s.jerkAbsMax,
                        "Validate");
          }
        });
    PhaseTiming validation;
    validation.name = "Validation";
    validation.wallS = std::chrono::duration<double>(std::chrono::steady_clock::now() - validationStart).count();
    validation.steps = metrics.steps;
    validation.simS = metrics.horizonS;
    timer.add(validation);

    // ------------------------------------------------------------------
    // Report
    // ------------------------------------------------------------------
    ControlReport report;
    report.fmuPath = fmuPath.string();
    report.tracePath = tracePath.string();
    if (livePlot) {
      report.livePlotUrl = livePlot->url();
    }
    report.plant = task.getPlantModel();
    report.noise = task.getNoise();
    report.pid = best.params;
    report.tauC = best.tauC;
    report.dt = kSimulationStepS;
    report.validationMaxDurationS = kValidationMaxDurationS;
    report.settleHoldS = kValidationSettleHoldS;
    report.metrics = metrics;
    report.phases = timer.phases();
    report.total = timer.total();
    printControlReport(std::cout, report);

    if (livePlot && options->keepPlot) {
      std::cout << "Live plot open at " << livePlot->url() << " — press Enter to exit.\n";
      std::string line;
      std::getline(std::cin, line);
    }
    return 0;  // `livePlot` stops the plotter here; the watchdog pipe covers any other exit
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }
}
