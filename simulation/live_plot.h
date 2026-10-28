#ifndef SIMULATION_LIVE_PLOT_H_
#define SIMULATION_LIVE_PLOT_H_

#include <memory>
#include <string>

// Launches tools/live_plot.py, the real-time UDP plot that the simulation
// streams into (replaces PlotJuggler).
//
// The plotter's lifetime is bound to this process: it is stopped when the
// handle is destroyed (normal exit), and it also watches a pipe that the
// kernel closes if this process dies in any other way (exception escape,
// Ctrl-C, kill -9), so it can never linger and hold the ports.

struct LivePlotOptions {
  std::string scriptPath;              // tools/live_plot.py
  std::string udpHost{"127.0.0.1"};
  int udpPort{9870};
  int httpPort{8088};                  // 0 = let the OS choose
  std::string title{"PID autotuning - live"};
  std::string logPath{"live_plot.log"};
  bool openBrowser{true};
  double startupTimeoutS{15.0};
  double stopTimeoutS{3.0};            // grace period before SIGKILL on stop()
};

class LivePlot {
 public:
  ~LivePlot();
  LivePlot(const LivePlot&) = delete;
  LivePlot& operator=(const LivePlot&) = delete;

  const std::string& url() const { return _url; }
  const std::string& logPath() const { return _logPath; }

  // Stops the plotter (idempotent). Called by the destructor.
  void stop();

 private:
  friend std::unique_ptr<LivePlot> startLivePlot(const LivePlotOptions&, std::string*);
  LivePlot(int pid, int parentPipeFd, std::string url, std::string logPath, double stopTimeoutS);

  int _pid;
  int _parentPipeFd;  // write end of the watchdog pipe; closing it tells the child to exit
  std::string _url;
  std::string _logPath;
  double _stopTimeoutS;
};

// Starts the plotter and waits until it reports that its sockets are bound.
// Returns nullptr (and leaves a message in `error`) if it could not be
// started; the simulation is expected to continue regardless.
std::unique_ptr<LivePlot> startLivePlot(const LivePlotOptions& options, std::string* error);

#endif  // SIMULATION_LIVE_PLOT_H_
