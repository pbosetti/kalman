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

//! \file
//! \brief Benchmarks one full predict/update cycle of each filter variant on
//!        the 3-DOF planar-robot models from examples/Robot1.

#include "lib/Benchmark.hpp"

// Defines Robot1::State and must precede the measurement-model headers below:
// the upstream models (unlike this fork's) are not self-contained. Kept in its
// own include block so clang-format does not reorder it after the others.
#include "SystemModel.hpp"

#include "OrientationMeasurementModel.hpp"
#include "PositionMeasurementModel.hpp"

#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/SquareRootExtendedKalmanFilter.hpp>
#include <kalman/SquareRootUnscentedKalmanFilter.hpp>
#include <kalman/UnscentedKalmanFilter.hpp>

#include <cstdlib>
#include <string>

namespace {

using T = float;

using State = KalmanExamples::Robot1::State<T>;
using Control = KalmanExamples::Robot1::Control<T>;

template <template <class> class CovarianceBase>
using SystemModel = KalmanExamples::Robot1::SystemModel<T, CovarianceBase>;
template <template <class> class CovarianceBase>
using PositionModel =
    KalmanExamples::Robot1::PositionMeasurementModel<T, CovarianceBase>;
template <template <class> class CovarianceBase>
using OrientationModel =
    KalmanExamples::Robot1::OrientationMeasurementModel<T, CovarianceBase>;

using PositionMeasurement = KalmanExamples::Robot1::PositionMeasurement<T>;
using OrientationMeasurement =
    KalmanExamples::Robot1::OrientationMeasurement<T>;

//! Run `iterations` full predict + orientation-update + position-update cycles
//! for a given filter and covariance representation. Measurements are taken
//! from the predicted state so the filter stays numerically well-behaved over
//! arbitrarily many iterations.
template <class Filter, template <class> class CovarianceBase>
void run_cycles(std::size_t iterations) {
  Filter filter;
  SystemModel<CovarianceBase> sys;
  PositionModel<CovarianceBase> pm(-10, -10, 30, 75);
  OrientationModel<CovarianceBase> om;

  State x;
  x.setZero();
  filter.init(x);

  Control u;
  u.v() = T(1);
  u.dtheta() = T(0.05);

  double acc = 0.0;
  for (std::size_t i = 0; i < iterations; ++i) {
    const State &x_pred = filter.predict(sys, u);

    OrientationMeasurement z_orient = om.h(x_pred);
    filter.update(om, z_orient);

    PositionMeasurement z_pos = pm.h(x_pred);
    const State &x_upd = filter.update(pm, z_pos);

    acc += x_upd.x() + x_upd.y() + x_upd.theta();
  }
  Kalman::Benchmarking::sink += acc;
}

} // namespace

BENCHMARK(EKF, iterations) {
  run_cycles<Kalman::ExtendedKalmanFilter<State>, Kalman::StandardBase>(
      iterations);
}

BENCHMARK(SR_EKF, iterations) {
  run_cycles<Kalman::SquareRootExtendedKalmanFilter<State>,
             Kalman::SquareRootBase>(iterations);
}

BENCHMARK(UKF, iterations) {
  run_cycles<Kalman::UnscentedKalmanFilter<State>, Kalman::StandardBase>(
      iterations);
}

BENCHMARK(SR_UKF, iterations) {
  run_cycles<Kalman::SquareRootUnscentedKalmanFilter<State>,
             Kalman::SquareRootBase>(iterations);
}

int main(int argc, char **argv) {
  std::size_t iterations = 200000;
  Kalman::Benchmarking::Format format = Kalman::Benchmarking::Format::Plain;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--markdown" || arg == "-m") {
      format = Kalman::Benchmarking::Format::Markdown;
    } else if (arg == "--csv") {
      format = Kalman::Benchmarking::Format::Csv;
    } else if (arg.rfind("--iterations=", 0) == 0) {
      iterations = static_cast<std::size_t>(std::strtoull(
          arg.c_str() + std::string("--iterations=").size(), nullptr, 10));
    }
  }

  return Kalman::Benchmarking::run_all(iterations, format);
}
