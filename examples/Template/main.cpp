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
//! \brief Minimal "hello world" template: a 1-D constant-velocity tracker.
//!
//! This is the smallest useful, fully self-contained example and is meant to be
//! copy-pasted as a starting point for your own filter. It estimates the
//! position and velocity of an object moving along a line from noisy position
//! measurements, using the Extended Kalman Filter.
//!
//! To adapt it to your problem you typically only change four things:
//!   1. the State vector,
//!   2. the system model f() (and its Jacobian F),
//!   3. the Measurement vector, and
//!   4. the measurement model h() (and its Jacobian H).

#include <kalman/ExtendedKalmanFilter.hpp>

#include <iostream>
#include <random>

using T = double;

// 1. State vector: [position, velocity].
using State = Kalman::Vector<T, 2>;

// 2. System model: a linear constant-velocity model
//        p_{k+1} = p_k + dt * v_k
//        v_{k+1} = v_k
//    Because the model is linear its Jacobian F is constant, so we set it once
//    in the constructor and do not need to override update_jacobians().
class SystemModel : public Kalman::LinearizedSystemModel<State> {
public:
  explicit SystemModel(T dt = T(1)) : _dt(dt) {
    this->F << T(1), dt, T(0), T(1);
  }

  State f(const State &x, const Control & /*u*/) const override {
    State next;
    next << x(0) + _dt * x(1), x(1);
    return next;
  }

private:
  T _dt;
};

// 3. Measurement vector: the measured position (1-D).
using Measurement = Kalman::Vector<T, 1>;

// 4. Measurement model: we observe the position directly, so h(x) = position
//    and the Jacobian H = [1, 0] is again constant.
class MeasurementModel
    : public Kalman::LinearizedMeasurementModel<State, Measurement> {
public:
  MeasurementModel() { this->H << T(1), T(0); }

  Measurement h(const State &x) const override {
    Measurement z;
    z << x(0);
    return z;
  }
};

int main() {
  SystemModel sys(/*dt=*/T(1));
  MeasurementModel mm;

  Kalman::ExtendedKalmanFilter<State> ekf;

  // Start at the origin, at rest.
  State x0;
  x0 << T(0), T(0);
  ekf.init(x0);

  // Simulate an object moving at a true velocity of 2 units/step and feed the
  // filter noisy position measurements.
  std::mt19937 rng(42);
  std::normal_distribution<T> noise(T(0), T(0.5));
  const T true_velocity = T(2);

  std::cout << "step  true_pos  meas_pos  est_pos  est_vel\n";
  T true_pos = T(0);
  for (int step = 1; step <= 10; ++step) {
    true_pos += true_velocity;

    // Predict the next state from the system model.
    ekf.predict(sys);

    // Correct using a noisy position measurement.
    Measurement z;
    z << true_pos + noise(rng);
    const State &x = ekf.update(mm, z);

    std::cout << "  " << step << "      " << true_pos << "       " << z(0)
              << "    " << x(0) << "   " << x(1) << "\n";
  }

  return 0;
}
