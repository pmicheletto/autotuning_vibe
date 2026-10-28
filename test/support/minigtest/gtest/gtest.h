// minigtest - a small GoogleTest-compatible test runner used when GoogleTest
// is not installed (offline builds). Same source syntax (TEST, EXPECT_*,
// ASSERT_*, streamed messages) and the same command-line protocol CTest's
// gtest_discover_tests() relies on (--gtest_list_tests, --gtest_filter).
// When GoogleTest is available the real library is used instead.

#ifndef MINIGTEST_GTEST_H_
#define MINIGTEST_GTEST_H_

#include <cmath>
#include <cstdio>
#include <limits>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

namespace testing {
namespace internal {

struct TestInfo {
  const char* suite;
  const char* name;
  void (*fn)();
};

std::vector<TestInfo>& registry();
bool registerTest(const char* suite, const char* name, void (*fn)());
void reportFailure(const char* file, int line, const std::string& summary, const std::string& message);

template <class T, class = void>
struct IsStreamable : std::false_type {};
template <class T>
struct IsStreamable<T, std::void_t<decltype(std::declval<std::ostream&>() << std::declval<const T&>())>>
    : std::true_type {};

template <class T>
std::string toString(const T& v) {
  if constexpr (std::is_same_v<T, bool>) {
    return v ? "true" : "false";
  } else if constexpr (std::is_enum_v<T>) {
    return std::to_string(static_cast<long long>(v));
  } else if constexpr (IsStreamable<T>::value) {
    std::ostringstream os;
    os.precision(17);
    os << v;
    return os.str();
  } else {
    return "<unprintable>";
  }
}

template <class A, class B>
std::string describe(const char* exprA, const char* exprB, const A& a, const B& b) {
  return std::string(exprA) + " = " + toString(a) + ", " + exprB + " = " + toString(b);
}

// Collects a streamed failure message; prints it when destroyed if the
// assertion failed.
class Failure {
 public:
  Failure(bool failed, const char* file, int line, std::string summary)
      : _failed(failed), _file(file), _line(line), _summary(std::move(summary)) {}
  ~Failure() {
    if (_failed) {
      reportFailure(_file, _line, _summary, _message.str());
    }
  }
  template <class T>
  Failure& operator<<(const T& v) {
    _message << v;
    return *this;
  }
  bool failed() const { return _failed; }

 private:
  bool _failed;
  const char* _file;
  int _line;
  std::string _summary;
  std::ostringstream _message;
};

inline bool almostEqual(double a, double b) {
  return std::fabs(a - b) <= 4.0 * std::numeric_limits<double>::epsilon() * std::fmax(std::fabs(a), std::fabs(b)) ||
         std::fabs(a - b) <= std::numeric_limits<double>::denorm_min();
}

}  // namespace internal
}  // namespace testing

#define TEST(suite, name)                                                                        \
  static void suite##_##name##_Test();                                                           \
  static const bool suite##_##name##_registered =                                                \
      ::testing::internal::registerTest(#suite, #name, &suite##_##name##_Test);                  \
  static void suite##_##name##_Test()

// EXPECT_*: record and continue. ASSERT_*: record and return from the test.
#define MINIGTEST_EXPECT_(cond, summary) ::testing::internal::Failure(!(cond), __FILE__, __LINE__, (summary))
#define MINIGTEST_ASSERT_(cond, summary)                                                         \
  if (!(cond)) return (void)::testing::internal::Failure(true, __FILE__, __LINE__, (summary));  \
  ::testing::internal::Failure(false, __FILE__, __LINE__, "")

#define MINIGTEST_CMP_(kind, a, b, op, opname)                                                   \
  MINIGTEST_##kind##_(((a) op (b)), std::string("Expected: " #a " " opname " " #b ", where ") +  \
                                        ::testing::internal::describe(#a, #b, (a), (b)))

#define EXPECT_TRUE(c) MINIGTEST_EXPECT_(!!(c), "Expected: " #c " is true")
#define EXPECT_FALSE(c) MINIGTEST_EXPECT_(!(c), "Expected: " #c " is false")
#define EXPECT_EQ(a, b) MINIGTEST_CMP_(EXPECT, a, b, ==, "==")
#define EXPECT_NE(a, b) MINIGTEST_CMP_(EXPECT, a, b, !=, "!=")
#define EXPECT_LT(a, b) MINIGTEST_CMP_(EXPECT, a, b, <, "<")
#define EXPECT_LE(a, b) MINIGTEST_CMP_(EXPECT, a, b, <=, "<=")
#define EXPECT_GT(a, b) MINIGTEST_CMP_(EXPECT, a, b, >, ">")
#define EXPECT_GE(a, b) MINIGTEST_CMP_(EXPECT, a, b, >=, ">=")
#define EXPECT_NEAR(a, b, tol)                                                                   \
  MINIGTEST_EXPECT_(std::fabs((a) - (b)) <= (tol),                                               \
                    std::string("Expected: |" #a " - " #b "| <= " #tol ", where ") +             \
                        ::testing::internal::describe(#a, #b, (a), (b)))
#define EXPECT_DOUBLE_EQ(a, b)                                                                   \
  MINIGTEST_EXPECT_(::testing::internal::almostEqual((a), (b)),                                  \
                    std::string("Expected: " #a " == " #b " (4 ULP), where ") +                  \
                        ::testing::internal::describe(#a, #b, (a), (b)))
#define EXPECT_FLOAT_EQ(a, b) EXPECT_NEAR(static_cast<double>(a), static_cast<double>(b), 4e-7 * std::fmax(std::fabs((double)(a)), 1.0))

#define ASSERT_TRUE(c) MINIGTEST_ASSERT_(!!(c), "Expected: " #c " is true")
#define ASSERT_FALSE(c) MINIGTEST_ASSERT_(!(c), "Expected: " #c " is false")
#define ASSERT_EQ(a, b) MINIGTEST_CMP_(ASSERT, a, b, ==, "==")
#define ASSERT_NE(a, b) MINIGTEST_CMP_(ASSERT, a, b, !=, "!=")
#define ASSERT_LT(a, b) MINIGTEST_CMP_(ASSERT, a, b, <, "<")
#define ASSERT_LE(a, b) MINIGTEST_CMP_(ASSERT, a, b, <=, "<=")
#define ASSERT_GT(a, b) MINIGTEST_CMP_(ASSERT, a, b, >, ">")
#define ASSERT_GE(a, b) MINIGTEST_CMP_(ASSERT, a, b, >=, ">=")
#define ASSERT_NEAR(a, b, tol)                                                                   \
  MINIGTEST_ASSERT_(std::fabs((a) - (b)) <= (tol),                                               \
                    std::string("Expected: |" #a " - " #b "| <= " #tol ", where ") +             \
                        ::testing::internal::describe(#a, #b, (a), (b)))

#define EXPECT_THROW(stmt, exc)                                                                  \
  do {                                                                                           \
    bool threw = false, right = false;                                                           \
    try { stmt; } catch (const exc&) { threw = right = true; } catch (...) { threw = true; }      \
    MINIGTEST_EXPECT_(right, threw ? "Expected: " #stmt " throws " #exc ", but threw another type" \
                                   : "Expected: " #stmt " throws " #exc ", but threw nothing");   \
  } while (0)
#define EXPECT_NO_THROW(stmt)                                                                    \
  do {                                                                                           \
    bool threw = false;                                                                          \
    try { stmt; } catch (...) { threw = true; }                                                  \
    MINIGTEST_EXPECT_(!threw, "Expected: " #stmt " does not throw, but it threw");               \
  } while (0)
#define EXPECT_ANY_THROW(stmt)                                                                   \
  do {                                                                                           \
    bool threw = false;                                                                          \
    try { stmt; } catch (...) { threw = true; }                                                  \
    MINIGTEST_EXPECT_(threw, "Expected: " #stmt " throws, but it did not");                      \
  } while (0)
#define ASSERT_NO_THROW(stmt)                                                                    \
  do {                                                                                           \
    bool threw = false;                                                                          \
    try { stmt; } catch (...) { threw = true; }                                                  \
    if (threw) return (void)::testing::internal::Failure(true, __FILE__, __LINE__,               \
                                                         "Expected: " #stmt " does not throw, but it threw"); \
  } while (0)
#define ASSERT_THROW(stmt, exc)                                                                  \
  do {                                                                                           \
    bool right = false;                                                                          \
    try { stmt; } catch (const exc&) { right = true; } catch (...) {}                            \
    if (!right) return (void)::testing::internal::Failure(true, __FILE__, __LINE__,              \
                                                          "Expected: " #stmt " throws " #exc);   \
  } while (0)

#endif  // MINIGTEST_GTEST_H_
