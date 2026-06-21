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
//! \brief Python bindings (pybind11) for the Unscented Kalman Filter.

#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>

#include "KalmanWrapper.hpp"

namespace py = pybind11;
using Kalman::Bindings::Filter;
using Kalman::Bindings::make_unscented;
using Kalman::Bindings::MeasurementFn;
using Kalman::Bindings::TransitionFn;

PYBIND11_MODULE(kalman, m) {
  m.doc() =
      "Python bindings for the Kalman filter library "
      "(https://github.com/pbosetti/kalman).\n\n"
      "Exposes a callback-driven Unscented Kalman Filter: supply f(x) and h(x) "
      "as plain Python callables operating on NumPy arrays.";

  m.attr("MAX_DIM") = KALMAN_BIND_MAX_DIM;

  py::class_<Filter>(m, "UnscentedKalmanFilter", R"doc(
Unscented Kalman Filter with run-time dimensions.

Parameters
----------
state_dim : int
    Number of state variables (1..MAX_DIM).
measurement_dim : int
    Number of measured variables (1..MAX_DIM).
f : Callable[[numpy.ndarray], numpy.ndarray]
    State-transition function mapping a state to the next state. Fold any
    control input into this callable.
h : Callable[[numpy.ndarray], numpy.ndarray]
    Measurement function mapping a state to an expected measurement.
)doc")
      .def(py::init([](int state_dim, int measurement_dim, TransitionFn f,
                       MeasurementFn h) {
             return make_unscented(state_dim, measurement_dim, std::move(f),
                                   std::move(h));
           }),
           py::arg("state_dim"), py::arg("measurement_dim"), py::arg("f"),
           py::arg("h"))
      .def_property_readonly("state_dim", &Filter::state_dim)
      .def_property_readonly("measurement_dim", &Filter::measurement_dim)
      .def_property("state", &Filter::state, &Filter::set_state,
                    "Current state estimate (NumPy array).")
      .def_property("covariance", &Filter::covariance, &Filter::set_covariance,
                    "Current state covariance (NumPy matrix).")
      .def("set_process_noise", &Filter::set_process_noise, py::arg("Q"),
           "Set the process-noise covariance Q (state_dim x state_dim).")
      .def("set_measurement_noise", &Filter::set_measurement_noise,
           py::arg("R"),
           "Set the measurement-noise covariance R "
           "(measurement_dim x measurement_dim).")
      .def("predict", &Filter::predict,
           "Perform the time-update step; returns the predicted state.")
      .def("update", &Filter::update, py::arg("z"),
           "Perform the measurement-update step with observation z; returns "
           "the corrected state.");
}
