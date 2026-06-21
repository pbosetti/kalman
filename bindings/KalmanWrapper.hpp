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
#ifndef KALMAN_BINDINGS_KALMANWRAPPER_HPP_
#define KALMAN_BINDINGS_KALMANWRAPPER_HPP_

//! \file
//! \brief Framework-agnostic, runtime-sized facade over the (compile-time
//!        sized) Unscented Kalman Filter, shared by the Python and R bindings.
//!
//! The core library is templated on the state/measurement dimensionality, which
//! must be known at compile time. Scripting languages need runtime dimensions,
//! so this facade instantiates the filter for every dimension pair up to
//! KALMAN_BIND_MAX_DIM and dispatches at run time behind a type-erased
//! interface (Kalman::Bindings::Filter). The Unscented filter is used because
//! it needs no Jacobians, so a binding only has to forward two callbacks:
//!
//!   - f(x) -> x'  the (possibly non-linear) state-transition function
//!   - h(x) -> z   the (possibly non-linear) measurement function
//!
//! Control inputs are not modelled explicitly; fold them into the f callback.

#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>

#include <kalman/UnscentedKalmanFilter.hpp>

//! Largest state/measurement dimension the bindings support at run time.
#ifndef KALMAN_BIND_MAX_DIM
#define KALMAN_BIND_MAX_DIM 6
#endif

namespace Kalman {
namespace Bindings {

using DynVector = Eigen::VectorXd;
using DynMatrix = Eigen::MatrixXd;

//! State-transition callback: maps a state vector to the next state vector.
using TransitionFn = std::function<DynVector(const DynVector &)>;
//! Measurement callback: maps a state vector to an expected measurement.
using MeasurementFn = std::function<DynVector(const DynVector &)>;

//! Type-erased, runtime-sized filter interface exposed to the bindings.
class Filter {
public:
  virtual ~Filter() = default;

  virtual int state_dim() const = 0;
  virtual int measurement_dim() const = 0;

  virtual void set_state(const DynVector &x) = 0;
  virtual DynVector state() const = 0;

  virtual void set_covariance(const DynMatrix &P) = 0;
  virtual DynMatrix covariance() const = 0;

  //! Process-noise covariance Q (state_dim x state_dim).
  virtual void set_process_noise(const DynMatrix &Q) = 0;
  //! Measurement-noise covariance R (measurement_dim x measurement_dim).
  virtual void set_measurement_noise(const DynMatrix &R) = 0;

  //! Time update.
  virtual DynVector predict() = 0;
  //! Measurement update with observation z.
  virtual DynVector update(const DynVector &z) = 0;
};

namespace detail {

inline void require(bool ok, const std::string &message) {
  if (!ok)
    throw std::invalid_argument(message);
}

//! Concrete fixed-size implementation for a (N state, M measurement) pair.
template <int N, int M> class FilterImpl final : public Filter {
  using State = Kalman::Vector<double, N>;
  using Measurement = Kalman::Vector<double, M>;

  //! System model that forwards f() to the user callback.
  class SystemModel : public Kalman::SystemModel<State> {
    using Base = Kalman::SystemModel<State>;

  public:
    using typename Base::Control;
    explicit SystemModel(TransitionFn fn) : _fn(std::move(fn)) {}
    State f(const State &x, const Control & /*u*/) const override {
      const DynVector next = _fn(DynVector(x));
      require(next.size() == N,
              "transition function returned a vector of the wrong size");
      State out;
      out = next;
      return out;
    }

  private:
    TransitionFn _fn;
  };

  //! Measurement model that forwards h() to the user callback.
  class MeasurementModel : public Kalman::MeasurementModel<State, Measurement> {
  public:
    explicit MeasurementModel(MeasurementFn fn) : _fn(std::move(fn)) {}
    Measurement h(const State &x) const override {
      const DynVector z = _fn(DynVector(x));
      require(z.size() == M,
              "measurement function returned a vector of the wrong size");
      Measurement out;
      out = z;
      return out;
    }

  private:
    MeasurementFn _fn;
  };

public:
  FilterImpl(TransitionFn f, MeasurementFn h)
      : _sys(std::move(f)), _measurement(std::move(h)) {}

  int state_dim() const override { return N; }
  int measurement_dim() const override { return M; }

  void set_state(const DynVector &x) override {
    require(x.size() == N, "state vector has the wrong size");
    State s;
    s = x;
    _ukf.init(s);
  }

  DynVector state() const override { return DynVector(_ukf.get_state()); }

  void set_covariance(const DynMatrix &P) override {
    require(P.rows() == N && P.cols() == N,
            "state covariance has the wrong shape");
    Kalman::Covariance<State> p;
    p = P;
    _ukf.set_covariance(p);
  }

  DynMatrix covariance() const override {
    return DynMatrix(_ukf.get_covariance());
  }

  void set_process_noise(const DynMatrix &Q) override {
    require(Q.rows() == N && Q.cols() == N,
            "process-noise covariance has the wrong shape");
    Kalman::Covariance<State> q;
    q = Q;
    _sys.set_covariance(q);
  }

  void set_measurement_noise(const DynMatrix &R) override {
    require(R.rows() == M && R.cols() == M,
            "measurement-noise covariance has the wrong shape");
    Kalman::Covariance<Measurement> r;
    r = R;
    _measurement.set_covariance(r);
  }

  DynVector predict() override { return DynVector(_ukf.predict(_sys)); }

  DynVector update(const DynVector &z) override {
    require(z.size() == M, "measurement vector has the wrong size");
    Measurement zz;
    zz = z;
    return DynVector(_ukf.update(_measurement, zz));
  }

private:
  Kalman::UnscentedKalmanFilter<State> _ukf;
  SystemModel _sys;
  MeasurementModel _measurement;
};

// Compile-time dispatch over the measurement dimension M (for a fixed N).
template <int N, int M> struct MeasurementDispatch {
  static std::unique_ptr<Filter> make(int m, TransitionFn &f,
                                      MeasurementFn &h) {
    if (m == M)
      return std::make_unique<FilterImpl<N, M>>(std::move(f), std::move(h));
    return MeasurementDispatch<N, M - 1>::make(m, f, h);
  }
};
template <int N> struct MeasurementDispatch<N, 0> {
  static std::unique_ptr<Filter> make(int, TransitionFn &, MeasurementFn &) {
    return nullptr;
  }
};

// Compile-time dispatch over the state dimension N.
template <int N> struct StateDispatch {
  static std::unique_ptr<Filter> make(int n, int m, TransitionFn &f,
                                      MeasurementFn &h) {
    if (n == N)
      return MeasurementDispatch<N, KALMAN_BIND_MAX_DIM>::make(m, f, h);
    return StateDispatch<N - 1>::make(n, m, f, h);
  }
};
template <> struct StateDispatch<0> {
  static std::unique_ptr<Filter> make(int, int, TransitionFn &,
                                      MeasurementFn &) {
    return nullptr;
  }
};

} // namespace detail

//! Build an Unscented Kalman Filter for the given runtime dimensions.
//!
//! @param state_dim       Number of state variables (1..KALMAN_BIND_MAX_DIM).
//! @param measurement_dim Number of measured variables
//! (1..KALMAN_BIND_MAX_DIM).
//! @param f               State-transition callback f(x) -> x'.
//! @param h               Measurement callback h(x) -> z.
//! @throws std::invalid_argument if a dimension is out of range.
inline std::unique_ptr<Filter> make_unscented(int state_dim,
                                              int measurement_dim,
                                              TransitionFn f, MeasurementFn h) {
  detail::require(state_dim >= 1 && state_dim <= KALMAN_BIND_MAX_DIM,
                  "state_dim out of range (1.." +
                      std::to_string(KALMAN_BIND_MAX_DIM) + ")");
  detail::require(measurement_dim >= 1 &&
                      measurement_dim <= KALMAN_BIND_MAX_DIM,
                  "measurement_dim out of range (1.." +
                      std::to_string(KALMAN_BIND_MAX_DIM) + ")");
  return detail::StateDispatch<KALMAN_BIND_MAX_DIM>::make(
      state_dim, measurement_dim, f, h);
}

} // namespace Bindings
} // namespace Kalman

#endif // KALMAN_BINDINGS_KALMANWRAPPER_HPP_
