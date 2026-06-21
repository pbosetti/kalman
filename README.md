# Kalman

[![CI](https://github.com/pbosetti/kalman/actions/workflows/ci.yml/badge.svg)](https://github.com/pbosetti/kalman/actions/workflows/ci.yml)
[![Benchmark](https://github.com/pbosetti/kalman/actions/workflows/benchmark.yml/badge.svg)](https://github.com/pbosetti/kalman/actions/workflows/benchmark.yml)
[![C++20](https://img.shields.io/badge/C%2B%2B-20-blue.svg)](https://en.cppreference.com/w/cpp/20)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE.txt)

**A fast, header-only C++20 library for [Kalman filtering](https://en.wikipedia.org/wiki/Kalman_filter) — built on [Eigen](https://eigen.tuxfamily.org), with zero hand-rolled linear algebra and zero install friction.**

Define a state vector, a system model, and a measurement model, and you have a
working state estimator in a few dozen lines. The library does the heavy lifting
(sigma points, Jacobian propagation, square-root covariance updates) behind a
small, strongly-typed interface.

> ### About this repository
> This is a **thorough, modernized review of the excellent original
> [mherb/kalman](https://github.com/mherb/kalman)** library. The filtering
> algorithms are faithfully preserved; everything *around* them has been
> reworked: a C++20 codebase, a dependency-free test and benchmark harness,
> Eigen pulled in automatically, first-class `FetchContent` support,
> cross-platform CI, and documentation aimed at getting you productive quickly.
> See [What's different](#whats-different-in-this-review) for the full list.

## Filter variants

| Variant | Class | Best for |
| :------ | :---- | :------- |
| Extended Kalman Filter | `Kalman::ExtendedKalmanFilter` | Mildly non-linear systems where Jacobians are easy to derive |
| Square-Root EKF | `Kalman::SquareRootExtendedKalmanFilter` | EKF problems needing better numerical conditioning |
| Unscented Kalman Filter | `Kalman::UnscentedKalmanFilter` | Strongly non-linear systems; no Jacobians required |
| Square-Root UKF | `Kalman::SquareRootUnscentedKalmanFilter` | The recommended UKF: same accuracy, more numerically robust |

## What's different in this review

Compared to the original [mherb/kalman](https://github.com/mherb/kalman):

- **C++20 throughout** — template parameters are constrained with
  [concepts](include/kalman/Concepts.hpp) (clear errors instead of deep template
  spew), plus `using` aliases, `[[nodiscard]]`, and `std::numbers`.
- **No external test framework** — GoogleTest is gone, replaced by a tiny
  bundled, header-only harness ([`test/lib/Test.hpp`](test/lib/Test.hpp)).
- **Eigen handled for you** — found on the system if present, otherwise fetched
  automatically (pinned release) via CMake `FetchContent`.
- **Drop-in consumption** — a single `Kalman::Kalman` CMake target that works
  identically via `FetchContent` and via `find_package` after install.
- **Cross-platform CI** — Linux (GCC/Clang), macOS (Clang) and Windows (MSVC),
  in Debug and Release.
- **A benchmark suite** that runs in CI and keeps the [results table](#performance)
  in this README up to date.
- **Consistent style** — LLVM `clang-format` and a documented naming convention.

## Requirements

- A C++20 compiler (tested with GCC 13, Clang 18 and MSVC 2022)
- CMake ≥ 3.16
- [Eigen 3](https://eigen.tuxfamily.org) ≥ 3.3 — used if installed, otherwise
  fetched automatically (Eigen 3.4.0). No manual setup required.

## Quick start

Pull the library straight into your CMake project with `FetchContent`:

```cmake
include(FetchContent)
FetchContent_Declare(
  kalman
  GIT_REPOSITORY https://github.com/pbosetti/kalman.git
  GIT_TAG        master)
FetchContent_MakeAvailable(kalman)

target_link_libraries(my_target PRIVATE Kalman::Kalman)
```

The `Kalman::Kalman` target propagates the include directories and the Eigen
dependency automatically. Alternatively, after `cmake --install`, locate it with
`find_package(Kalman REQUIRED)` — the same target name applies.

## A 60-second example

A complete 1-D constant-velocity tracker (see
[`examples/Template/main.cpp`](examples/Template/main.cpp) for the fully
commented version):

```cpp
#include <kalman/ExtendedKalmanFilter.hpp>

using T = double;
using State = Kalman::Vector<T, 2>; // [position, velocity]

// Constant-velocity model; F is constant, so set it once.
struct SystemModel : Kalman::LinearizedSystemModel<State> {
  T dt = 1;
  SystemModel() { this->_F << 1, dt, 0, 1; }
  State f(const State &x, const Control &) const override {
    State n; n << x(0) + dt * x(1), x(1); return n;
  }
};

using Measurement = Kalman::Vector<T, 1>; // measured position
struct MeasurementModel
    : Kalman::LinearizedMeasurementModel<State, Measurement> {
  MeasurementModel() { this->_H << 1, 0; }
  Measurement h(const State &x) const override {
    Measurement z; z << x(0); return z;
  }
};

int main() {
  SystemModel sys;
  MeasurementModel mm;
  Kalman::ExtendedKalmanFilter<State> ekf;

  State x0; x0 << 0, 0;
  ekf.init(x0);

  Measurement z; z << 2.1;        // a noisy position reading
  ekf.predict(sys);               // time update
  const State &x = ekf.update(mm, z); // measurement update
}
```

## Building and running the tests

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

Useful options (all default ON when Kalman is the top-level project and OFF when
it is consumed as a subproject):

| Option | Description |
| :----- | :---------- |
| `KALMAN_BUILD_TESTS` | Build the unit tests |
| `KALMAN_BUILD_EXAMPLES` | Build the example programs |
| `KALMAN_BUILD_BENCHMARKS` | Build the benchmark suite |
| `KALMAN_INSTALL` | Generate install/export rules |
| `KALMAN_USE_SYSTEM_EIGEN` | Prefer a system Eigen over fetching one |

## Adapting it to your own problem

Estimation with this library always comes down to four pieces:

1. **State vector** — what you want to estimate. Use `Kalman::Vector<T, N>` or
   derive your own named type with the `KALMAN_VECTOR` helper.
2. **System model** — how the state evolves. Derive from `Kalman::SystemModel`
   (non-linear filters) or `Kalman::LinearizedSystemModel` (EKF/SR-EKF, where you
   also provide the Jacobian) and implement `f()`.
3. **Measurement vector** — what your sensors report.
4. **Measurement model** — how the state maps to a measurement. Derive from
   `Kalman::MeasurementModel` / `Kalman::LinearizedMeasurementModel` and
   implement `h()`.

Two worked examples are included:

- [`examples/Template`](examples/Template/main.cpp) — the minimal starting point
  to copy from.
- [`examples/Robot1`](examples/Robot1) — a 3-DOF planar robot with a non-linear
  motion model and two different sensors (range-to-landmarks and a compass),
  compared across the EKF and UKF.

## Python and R bindings

Prefer to drive the filter from Python or R? Callback-based bindings to the
Unscented Kalman Filter are provided under [`bindings/`](bindings/) — supply
`f(x)` and `h(x)` in your language and the C++ core does the rest:

```python
import numpy as np, kalman
ukf = kalman.UnscentedKalmanFilter(
    state_dim=2, measurement_dim=1,
    f=lambda x: np.array([x[0] + x[1], x[1]]),
    h=lambda x: np.array([x[0]]))
ukf.predict(); ukf.update(np.array([2.1]))
```

See [`bindings/README.md`](bindings/README.md) for the Python (pybind11) and R
(Rcpp) build and usage instructions. Both are exercised in CI.

## Performance

One full *predict + measurement-update* cycle of the Robot1 model (3-D state,
`float`), measured by the bundled benchmark suite and compared against the
original [mherb/kalman](https://github.com/mherb/kalman) upstream. Reproduce
locally with:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target kalman_benchmark
./build/benchmark/kalman_benchmark --markdown
```

<!-- BENCHMARK:START -->
_Last updated: 2026-06-21 21:38 UTC. Indicative numbers from the CI runner; absolute values vary with hardware._

| Filter | Time / cycle | Throughput |
| :----- | -----------: | ---------: |
| EKF | 0.099 µs | 10.13M cycles/s |
| SR_EKF | 1.175 µs | 0.85M cycles/s |
| UKF | 0.210 µs | 4.76M cycles/s |
| SR_UKF | 0.978 µs | 1.02M cycles/s |
<!-- BENCHMARK:END -->

## Coding style and conventions

Code is formatted with `clang-format` (LLVM base; see [`.clang-format`](.clang-format))
and follows these naming conventions:

- **Classes, namespaces and type aliases**: `PascalCase`
- **Member functions**: `snake_case`
- **Member variables**: leading underscore; canonical Kalman math casing is
  preserved for single-symbol matrices/vectors (`_x`, `_P`, `_S`, `_F`, `_W`,
  `_H`, `_V`) and `snake_case` otherwise (`_sigma_state_points`).

The `Kalman::Cholesky` helper intentionally keeps Eigen's camelCase, as it
extends `Eigen::LLT`.

## Documentation

The headers are documented with Doxygen. Generate the HTML docs with:

```sh
doxygen Doxyfile
```

## Acknowledgements

This project is a modernized review of
[**mherb/kalman**](https://github.com/mherb/kalman) by Markus Herb. All credit
for the original design and the filter implementations goes to the upstream
authors. The unscented variants are based on *The square-root unscented Kalman
filter for state and parameter-estimation* by Rudolph van der Merwe and Eric A.
Wan.

## License

Released under the MIT License — see [LICENSE.txt](LICENSE.txt).

Copyright (c) 2015 [mherb](https://github.com/mherb) and contributors.
