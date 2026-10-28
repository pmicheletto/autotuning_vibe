#include "live_plot.h"

#include <fcntl.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <optional>
#include <thread>
#include <vector>

namespace {

constexpr const char* kReadyToken = "LIVEPLOT_READY";

// Reads the URL from the handshake line the plotter writes to its log:
//   LIVEPLOT_READY http://127.0.0.1:8088/ udp=127.0.0.1:9870
std::optional<std::string> readyUrlFromLog(const std::string& logPath) {
  std::ifstream log(logPath);
  if (!log) {
    return std::nullopt;
  }
  std::string line;
  while (std::getline(log, line)) {
    const auto pos = line.find(kReadyToken);
    if (pos == std::string::npos) {
      continue;
    }
    std::string rest = line.substr(pos + std::char_traits<char>::length(kReadyToken));
    const auto start = rest.find_first_not_of(" \t");
    if (start == std::string::npos) {
      continue;
    }
    rest = rest.substr(start);
    return rest.substr(0, rest.find_first_of(" \t\r\n"));
  }
  return std::nullopt;
}

// Returns true once the child has been reaped.
bool reaped(pid_t pid) {
  int status = 0;
  return waitpid(pid, &status, WNOHANG) == pid;
}

bool waitForExit(pid_t pid, double timeoutS) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(timeoutS);
  while (std::chrono::steady_clock::now() < deadline) {
    if (reaped(pid)) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return reaped(pid);
}

}  // namespace

LivePlot::LivePlot(int pid, int parentPipeFd, std::string url, std::string logPath,
                   double stopTimeoutS)
    : _pid(pid),
      _parentPipeFd(parentPipeFd),
      _url(std::move(url)),
      _logPath(std::move(logPath)),
      _stopTimeoutS(stopTimeoutS) {}

LivePlot::~LivePlot() { stop(); }

void LivePlot::stop() {
  if (_pid <= 0) {
    return;
  }
  // Closing the watchdog pipe is the same signal the child gets if we crash;
  // SIGTERM just makes the normal path faster.
  if (_parentPipeFd >= 0) {
    close(_parentPipeFd);
    _parentPipeFd = -1;
  }
  kill(_pid, SIGTERM);
  if (!waitForExit(_pid, _stopTimeoutS)) {
    kill(_pid, SIGKILL);
    waitpid(_pid, nullptr, 0);
  }
  _pid = 0;
}

std::unique_ptr<LivePlot> startLivePlot(const LivePlotOptions& options, std::string* error) {
  const auto fail = [error](const std::string& message) -> std::unique_ptr<LivePlot> {
    if (error) {
      *error = message;
    }
    return nullptr;
  };

  std::error_code ec;
  if (!std::filesystem::is_regular_file(options.scriptPath, ec)) {
    return fail("plot script not found: " + options.scriptPath);
  }

  // Truncate the log so the handshake we read belongs to this run.
  {
    std::ofstream truncate(options.logPath, std::ios::trunc);
    if (!truncate) {
      return fail("cannot write log file: " + options.logPath);
    }
  }

  // Watchdog pipe: the child reads its stdin until EOF, which happens when the
  // write end (held only by this process) is closed — explicitly in stop(), or
  // by the kernel when this process dies. O_CLOEXEC keeps the write end from
  // leaking into the child (which would defeat the EOF).
  int pipeFds[2];
  if (pipe2(pipeFds, O_CLOEXEC) != 0) {
    return fail(std::string("pipe failed: ") + strerror(errno));
  }
  const int readEnd = pipeFds[0];
  const int writeEnd = pipeFds[1];

  const std::string udpPort = std::to_string(options.udpPort);
  const std::string httpPort = std::to_string(options.httpPort);
  std::vector<std::string> args{"python3",
                                options.scriptPath,
                                "--udp-host", options.udpHost,
                                "--udp-port", udpPort,
                                "--http-port", httpPort,
                                "--title", options.title,
                                "--exit-with-parent"};
  if (!options.openBrowser) {
    args.emplace_back("--headless");
  }

  std::vector<char*> argv;
  argv.reserve(args.size() + 1);
  for (std::string& arg : args) {
    argv.push_back(arg.data());
  }
  argv.push_back(nullptr);

  const pid_t pid = fork();
  if (pid < 0) {
    close(readEnd);
    close(writeEnd);
    return fail("fork failed");
  }
  if (pid == 0) {
    // Child: own session (terminal Ctrl-C reaches it via the pipe, not SIGINT),
    // stdin = watchdog pipe, stdout/stderr = log file.
    setsid();
    dup2(readEnd, STDIN_FILENO);  // dup2 clears O_CLOEXEC on the new fd
    close(readEnd);
    const int fd = open(options.logPath.c_str(), O_WRONLY | O_APPEND);
    if (fd >= 0) {
      dup2(fd, STDOUT_FILENO);
      dup2(fd, STDERR_FILENO);
      if (fd > STDERR_FILENO) {
        close(fd);
      }
    }
    execvp(argv[0], argv.data());
    // execvp only returns on failure (e.g. no python3 on PATH); the message
    // lands in the log file that stderr was redirected to.
    dprintf(STDERR_FILENO, "live_plot: cannot execute '%s': %s\n", argv[0], strerror(errno));
    _exit(127);
  }

  // Parent: keep only the write end; wait for the handshake line in the log.
  close(readEnd);
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(options.startupTimeoutS);
  while (std::chrono::steady_clock::now() < deadline) {
    if (auto url = readyUrlFromLog(options.logPath)) {
      return std::unique_ptr<LivePlot>(
          new LivePlot(static_cast<int>(pid), writeEnd, *url, options.logPath, options.stopTimeoutS));
    }
    if (reaped(pid)) {
      close(writeEnd);
      return fail("plot process exited early (see " + options.logPath + ")");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  close(writeEnd);
  kill(pid, SIGTERM);
  waitpid(pid, nullptr, 0);
  return fail("plot process did not start within timeout (see " + options.logPath + ")");
}
