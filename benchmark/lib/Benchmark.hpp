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
#ifndef KALMAN_BENCHMARKING_BENCHMARK_HPP_
#define KALMAN_BENCHMARKING_BENCHMARK_HPP_

//! \file
//! \brief Minimal, dependency-free micro-benchmark harness.
//!
//! Mirrors the philosophy of the bundled test harness: no external framework.
//! A benchmark is a function that performs `iterations` units of work; the
//! runner times it with a steady clock and reports the per-iteration cost.
//! Results can be emitted as a GitHub-flavoured Markdown table (consumed by CI
//! to refresh the table in the README) or as plain text.

#include <chrono>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace Kalman {
namespace Benchmarking {

//! Signature of a benchmark body: it must perform `iterations` units of work.
using BenchmarkFn = void (*)(std::size_t iterations);

//! A single registered benchmark.
struct BenchmarkCase {
  const char *name;
  BenchmarkFn fn;
};

//! Global registry (Meyers singleton, shared across translation units).
inline std::vector<BenchmarkCase> &registry() {
  static std::vector<BenchmarkCase> cases;
  return cases;
}

//! Registers a benchmark at static-initialization time.
struct Registrar {
  Registrar(const char *name, BenchmarkFn fn) {
    registry().push_back(BenchmarkCase{name, fn});
  }
};

//! Sink used to defeat dead-code elimination in a portable way (no inline asm,
//! so it also works under MSVC). Benchmarks accumulate a representative scalar
//! here; the runner reads it at the end.
inline volatile double sink = 0.0;

//! Run every registered benchmark and print a report.
//!
//! @param iterations Units of work per benchmark (per-iteration cost = total
//!                   time / iterations).
//! @param markdown   Emit a GitHub-flavoured Markdown table when true.
//! @return Process exit code (always 0; benchmarks do not assert).
inline int run_all(std::size_t iterations, bool markdown) {
  std::vector<BenchmarkCase> &cases = registry();

  struct Row {
    std::string name;
    double ns_per_iter;
  };
  std::vector<Row> rows;

  for (std::size_t i = 0; i < cases.size(); ++i) {
    // Warm up (also primes caches / branch predictors).
    cases[i].fn(iterations / 10 + 1);

    const auto start = std::chrono::steady_clock::now();
    cases[i].fn(iterations);
    const auto end = std::chrono::steady_clock::now();

    const double total_ns =
        std::chrono::duration<double, std::nano>(end - start).count();
    rows.push_back(
        Row{cases[i].name, total_ns / static_cast<double>(iterations)});
  }

  std::ostringstream out;
  if (markdown) {
    out << "| Filter | Time / cycle | Throughput |\n";
    out << "| :----- | -----------: | ---------: |\n";
    for (const Row &r : rows) {
      out << "| " << r.name << " | " << std::fixed << std::setprecision(3)
          << (r.ns_per_iter / 1000.0) << " \xC2\xB5s | " << std::fixed
          << std::setprecision(2) << (1000.0 / r.ns_per_iter)
          << "M cycles/s |\n";
    }
  } else {
    out << "Benchmark results (" << iterations << " iterations each):\n";
    for (const Row &r : rows) {
      out << "  " << std::left << std::setw(28) << r.name << std::right
          << std::fixed << std::setprecision(1) << r.ns_per_iter
          << " ns/cycle\n";
    }
  }

  std::cout << out.str();
  // Touch the sink so the optimizer cannot discard the measured work.
  if (sink == 1234.56789)
    std::cout << "";
  return 0;
}

} // namespace Benchmarking
} // namespace Kalman

//! Define and auto-register a benchmark. The body receives `iterations` and
//! must perform that many units of work, e.g.:
//! \code
//!   BENCHMARK(MyFilter, iterations) {
//!     for (std::size_t i = 0; i < iterations; ++i) { ... }
//!   }
//! \endcode
#define BENCHMARK(bench_name, iterations_param)                                \
  static void bench_name##_body(std::size_t iterations_param);                 \
  static ::Kalman::Benchmarking::Registrar bench_name##_registrar(             \
      #bench_name, &bench_name##_body);                                        \
  static void bench_name##_body(std::size_t iterations_param)

#endif // KALMAN_BENCHMARKING_BENCHMARK_HPP_
