// minigtest runner: GoogleTest-compatible command line and output.
//   --gtest_list_tests            print "Suite.\n  Test" for every test
//   --gtest_filter=PATTERNS       run tests matching POS[:POS...][-NEG[:NEG...]]
//                                 ('*' and '?' wildcards, as in GoogleTest)
// Other --gtest_* flags are accepted and ignored.

#include <gtest/gtest.h>

#include <chrono>
#include <cstring>
#include <string>
#include <vector>

namespace testing {
namespace internal {

std::vector<TestInfo>& registry() {
  static std::vector<TestInfo> tests;
  return tests;
}

bool registerTest(const char* suite, const char* name, void (*fn)()) {
  registry().push_back(TestInfo{suite, name, fn});
  return true;
}

namespace {
int g_failuresInCurrentTest = 0;
}  // namespace

void reportFailure(const char* file, int line, const std::string& summary, const std::string& message) {
  ++g_failuresInCurrentTest;
  std::printf("%s:%d: Failure\n%s\n", file, line, summary.c_str());
  if (!message.empty()) {
    std::printf("%s\n", message.c_str());
  }
}

}  // namespace internal
}  // namespace testing

namespace {

bool globMatch(const char* pattern, const char* text) {
  if (*pattern == '\0') return *text == '\0';
  if (*pattern == '*') return globMatch(pattern + 1, text) || (*text != '\0' && globMatch(pattern, text + 1));
  if (*text == '\0') return false;
  return (*pattern == '?' || *pattern == *text) && globMatch(pattern + 1, text + 1);
}

std::vector<std::string> split(const std::string& s, char sep) {
  std::vector<std::string> out;
  std::string cur;
  for (char c : s) {
    if (c == sep) {
      if (!cur.empty()) out.push_back(cur);
      cur.clear();
    } else {
      cur += c;
    }
  }
  if (!cur.empty()) out.push_back(cur);
  return out;
}

// GoogleTest filter semantics: positive patterns, optionally followed by
// '-' and negative patterns; an empty positive part means "all".
bool filterMatches(const std::string& filter, const std::string& fullName) {
  const auto dash = filter.find('-');
  const std::vector<std::string> positive = split(filter.substr(0, dash), ':');
  const std::vector<std::string> negative =
      dash == std::string::npos ? std::vector<std::string>{} : split(filter.substr(dash + 1), ':');
  bool matched = positive.empty();
  for (const std::string& p : positive) matched = matched || globMatch(p.c_str(), fullName.c_str());
  for (const std::string& n : negative) matched = matched && !globMatch(n.c_str(), fullName.c_str());
  return matched;
}

}  // namespace

int main(int argc, char** argv) {
  using testing::internal::registry;
  bool listOnly = false;
  std::string filter = "*";
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--gtest_list_tests") == 0) {
      listOnly = true;
    } else if (std::strncmp(argv[i], "--gtest_filter=", 15) == 0) {
      filter = argv[i] + 15;
    }
  }

  std::vector<const testing::internal::TestInfo*> selected;
  for (const auto& t : registry()) {
    if (filterMatches(filter, std::string(t.suite) + "." + t.name)) {
      selected.push_back(&t);
    }
  }

  if (listOnly) {
    const char* currentSuite = "";
    for (const auto* t : selected) {
      if (std::strcmp(currentSuite, t->suite) != 0) {
        std::printf("%s.\n", t->suite);
        currentSuite = t->suite;
      }
      std::printf("  %s\n", t->name);
    }
    return 0;
  }

  std::printf("[==========] Running %zu test(s) (minigtest).\n", selected.size());
  std::vector<std::string> failed;
  const auto start = std::chrono::steady_clock::now();
  for (const auto* t : selected) {
    const std::string name = std::string(t->suite) + "." + t->name;
    std::printf("[ RUN      ] %s\n", name.c_str());
    testing::internal::g_failuresInCurrentTest = 0;
    try {
      t->fn();
    } catch (const std::exception& e) {
      testing::internal::reportFailure("<test body>", 0, "Unexpected exception: " + std::string(e.what()), "");
    } catch (...) {
      testing::internal::reportFailure("<test body>", 0, "Unexpected non-standard exception", "");
    }
    if (testing::internal::g_failuresInCurrentTest == 0) {
      std::printf("[       OK ] %s\n", name.c_str());
    } else {
      std::printf("[  FAILED  ] %s\n", name.c_str());
      failed.push_back(name);
    }
  }
  const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  std::printf("[==========] %zu test(s) ran (%.0f ms total).\n[  PASSED  ] %zu test(s).\n",
              selected.size(), ms, selected.size() - failed.size());
  for (const std::string& name : failed) {
    std::printf("[  FAILED  ] %s\n", name.c_str());
  }
  return failed.empty() ? 0 : 1;
}
