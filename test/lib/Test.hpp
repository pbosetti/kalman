// The MIT License (MIT)
//
// Copyright (c) 2015 Markus Herb
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
#ifndef KALMAN_TESTING_TEST_HPP_
#define KALMAN_TESTING_TEST_HPP_

//! \file
//! \brief Minimal, dependency-free unit-test harness for the Kalman library.
//!
//! This header provides a tiny GoogleTest-compatible subset that is sufficient
//! to drive the library's unit tests without pulling in any external testing
//! framework. It supports:
//!
//!  - Test registration through the `TEST(suite, name)` macro
//!  - Fatal assertions (`ASSERT_*`) that abort the current test on failure
//!  - GoogleTest-style ULP-based floating point comparison
//!    (`ASSERT_FLOAT_EQ` / `ASSERT_DOUBLE_EQ`)
//!
//! The public macro and assertion names intentionally mirror GoogleTest so
//! that existing tests can be ported with a single include change.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace Kalman {
namespace testing {

//! Exception thrown by a failing fatal assertion in order to abort the
//! currently running test case. It is caught by the test runner.
struct AssertionFailure : public std::exception {
  std::string message;
  explicit AssertionFailure(std::string msg) : message(std::move(msg)) {}
  const char *what() const noexcept override { return message.c_str(); }
};

//! Signature of a registered test-case body.
typedef void (*TestFn)();

//! A single registered test case.
struct TestCase {
  const char *suite;
  const char *name;
  TestFn fn;
};

//! Global registry of all test cases (Meyers singleton so that it is shared
//! across translation units).
inline std::vector<TestCase> &registry() {
  static std::vector<TestCase> tests;
  return tests;
}

//! Helper whose constructor registers a test case. One static instance is
//! emitted per `TEST(...)` definition.
struct Registrar {
  Registrar(const char *suite, const char *name, TestFn fn) {
    registry().push_back(TestCase{suite, name, fn});
  }
};

//! Maps a type to an unsigned integer of identical size.
template <std::size_t Size> struct TypeWithSize;
template <> struct TypeWithSize<4> {
  typedef std::uint32_t UInt;
};
template <> struct TypeWithSize<8> {
  typedef std::uint64_t UInt;
};

//! GoogleTest-style ULP based floating point comparison.
//!
//! Two finite numbers are considered "almost equal" when there are at most
//! four representable values between them (4 units in the last place). This
//! mirrors the behaviour of GoogleTest's `ASSERT_FLOAT_EQ` / `ASSERT_DOUBLE_EQ`
//! and is robust against the small rounding errors produced by the linear
//! algebra routines under test.
template <typename RawType> class FloatingPoint {
public:
  typedef typename TypeWithSize<sizeof(RawType)>::UInt Bits;

  static const std::size_t kBitCount = 8 * sizeof(RawType);
  static const std::size_t kFractionBitCount =
      std::numeric_limits<RawType>::digits - 1;
  static const std::size_t kExponentBitCount =
      kBitCount - 1 - kFractionBitCount;
  static const Bits kSignBitMask = static_cast<Bits>(1) << (kBitCount - 1);
  static const Bits kFractionBitMask = ~static_cast<Bits>(0) >>
                                       (kExponentBitCount + 1);
  static const Bits kExponentBitMask = ~(kSignBitMask | kFractionBitMask);
  static const std::size_t kMaxUlps = 4;

  explicit FloatingPoint(const RawType &x) { u_.value = x; }

  bool AlmostEquals(const FloatingPoint &rhs) const {
    if (isNan() || rhs.isNan())
      return false;
    return distance(u_.bits, rhs.u_.bits) <= kMaxUlps;
  }

private:
  union Union {
    RawType value;
    Bits bits;
  };

  bool isNan() const {
    return ((kExponentBitMask & u_.bits) == kExponentBitMask) &&
           ((kFractionBitMask & u_.bits) != 0);
  }

  static Bits toBiased(const Bits &sam) {
    if (kSignBitMask & sam)
      return ~sam + 1;
    return kSignBitMask | sam;
  }

  static Bits distance(const Bits &a, const Bits &b) {
    const Bits biasedA = toBiased(a);
    const Bits biasedB = toBiased(b);
    return (biasedA >= biasedB) ? (biasedA - biasedB) : (biasedB - biasedA);
  }

  Union u_;
};

//! Run every registered test case, printing a GoogleTest-like report.
//! \return 0 if all tests passed, 1 otherwise (suitable as a process exit code)
inline int runAll() {
  std::vector<TestCase> &tests = registry();
  std::size_t passed = 0;
  std::vector<std::string> failures;

  std::cout << "[==========] Running " << tests.size() << " test(s)\n";
  for (std::size_t i = 0; i < tests.size(); ++i) {
    const TestCase &tc = tests[i];
    std::cout << "[ RUN      ] " << tc.suite << "." << tc.name << "\n";

    bool ok = true;
    std::string detail;
    try {
      tc.fn();
    } catch (const AssertionFailure &f) {
      ok = false;
      detail = f.what();
    } catch (const std::exception &e) {
      ok = false;
      detail = std::string("unexpected exception: ") + e.what();
    } catch (...) {
      ok = false;
      detail = "unknown exception";
    }

    if (ok) {
      std::cout << "[       OK ] " << tc.suite << "." << tc.name << "\n";
      ++passed;
    } else {
      std::cout << "  " << detail << "\n";
      std::cout << "[  FAILED  ] " << tc.suite << "." << tc.name << "\n";
      std::ostringstream oss;
      oss << tc.suite << "." << tc.name;
      failures.push_back(oss.str());
    }
  }

  std::cout << "[==========] " << passed << " / " << tests.size()
            << " test(s) passed\n";
  if (!failures.empty()) {
    std::cout << "[  FAILED  ] " << failures.size() << " test(s):\n";
    for (std::size_t i = 0; i < failures.size(); ++i)
      std::cout << "             " << failures[i] << "\n";
  }
  return failures.empty() ? 0 : 1;
}

} // namespace testing
} // namespace Kalman

//! \name Test registration and assertion macros
//! \{

//! Define and auto-register a test case. Usage mirrors GoogleTest:
//! \code
//!   TEST(SuiteName, TestName) { ASSERT_TRUE(...); }
//! \endcode
#define TEST(suite_name, test_name)                                            \
  static void suite_name##_##test_name##_body();                               \
  static ::Kalman::testing::Registrar suite_name##_##test_name##_registrar(    \
      #suite_name, #test_name, &suite_name##_##test_name##_body);              \
  static void suite_name##_##test_name##_body()

//! Abort the current test with a formatted message. `msg_expr` is streamed,
//! so it may use the `<<` operator (e.g. `KALMAN_TEST_FAIL("x=" << x)`).
#define KALMAN_TEST_FAIL(msg_expr)                                             \
  do {                                                                         \
    std::ostringstream kalman_test_oss;                                        \
    kalman_test_oss << "Failure at " << __FILE__ << ":" << __LINE__            \
                    << "\n    " << msg_expr;                                   \
    throw ::Kalman::testing::AssertionFailure(kalman_test_oss.str());          \
  } while (0)

#define ASSERT_TRUE(cond)                                                      \
  do {                                                                         \
    if (!(cond))                                                               \
      KALMAN_TEST_FAIL("Expected true: " #cond);                               \
  } while (0)

#define ASSERT_FALSE(cond)                                                     \
  do {                                                                         \
    if ((cond))                                                                \
      KALMAN_TEST_FAIL("Expected false: " #cond);                              \
  } while (0)

#define ASSERT_EQ(a, b)                                                        \
  do {                                                                         \
    auto kalman_a = (a);                                                       \
    auto kalman_b = (b);                                                       \
    if (!(kalman_a == kalman_b))                                               \
      KALMAN_TEST_FAIL("Expected " #a " == " #b " (" << kalman_a << " vs "     \
                                                     << kalman_b << ")");      \
  } while (0)

#define ASSERT_NE(a, b)                                                        \
  do {                                                                         \
    auto kalman_a = (a);                                                       \
    auto kalman_b = (b);                                                       \
    if (!(kalman_a != kalman_b))                                               \
      KALMAN_TEST_FAIL("Expected " #a " != " #b " (" << kalman_a << " vs "     \
                                                     << kalman_b << ")");      \
  } while (0)

#define ASSERT_NEAR(a, b, tol)                                                 \
  do {                                                                         \
    double kalman_a = static_cast<double>(a);                                  \
    double kalman_b = static_cast<double>(b);                                  \
    if (std::abs(kalman_a - kalman_b) > (tol))                                 \
      KALMAN_TEST_FAIL("Expected |" #a " - " #b "| <= " #tol                   \
                       << " (" << kalman_a << " vs " << kalman_b << ")");      \
  } while (0)

#define ASSERT_FLOAT_EQ(a, b)                                                  \
  do {                                                                         \
    float kalman_a = static_cast<float>(a);                                    \
    float kalman_b = static_cast<float>(b);                                    \
    if (!::Kalman::testing::FloatingPoint<float>(kalman_a).AlmostEquals(       \
            ::Kalman::testing::FloatingPoint<float>(kalman_b)))                \
      KALMAN_TEST_FAIL("Expected " #a " ~= " #b " (" << kalman_a << " vs "     \
                                                     << kalman_b << ")");      \
  } while (0)

#define ASSERT_DOUBLE_EQ(a, b)                                                 \
  do {                                                                         \
    double kalman_a = static_cast<double>(a);                                  \
    double kalman_b = static_cast<double>(b);                                  \
    if (!::Kalman::testing::FloatingPoint<double>(kalman_a).AlmostEquals(      \
            ::Kalman::testing::FloatingPoint<double>(kalman_b)))               \
      KALMAN_TEST_FAIL("Expected " #a " ~= " #b " (" << kalman_a << " vs "     \
                                                     << kalman_b << ")");      \
  } while (0)

//! Assert that two Eigen matrices have identical shape and element-wise differ
//! by at most `delta`.
#define ASSERT_MATRIX_NEAR(A, B, delta)                                        \
  do {                                                                         \
    ASSERT_EQ((A).rows(), (B).rows());                                         \
    ASSERT_EQ((A).cols(), (B).cols());                                         \
    for (int kalman_i = 0; kalman_i < (A).rows(); ++kalman_i)                  \
      for (int kalman_j = 0; kalman_j < (A).cols(); ++kalman_j)                \
        ASSERT_NEAR((A)(kalman_i, kalman_j), (B)(kalman_i, kalman_j), delta);  \
  } while (0)

//! \}

#endif // KALMAN_TESTING_TEST_HPP_
