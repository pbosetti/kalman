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
#ifndef KALMAN_EXTENDEDKALMANFILTER_HPP_
#define KALMAN_EXTENDEDKALMANFILTER_HPP_

#include "KalmanFilterBase.hpp"
#include "LinearizedMeasurementModel.hpp"
#include "LinearizedSystemModel.hpp"
#include "StandardFilterBase.hpp"

namespace Kalman {

/**
 * @brief Extended Kalman Filter (EKF)
 *
 * This implementation is based upon [An Introduction to the Kalman
 * Filter](https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf) by Greg
 * Welch and Gary Bishop.
 *
 * @param StateType The vector-type of the system state (usually some type
 * derived from Kalman::Vector)
 */
template <class StateType>
class ExtendedKalmanFilter : public KalmanFilterBase<StateType>,
                             public StandardFilterBase<StateType> {
public:
  //! Kalman Filter base type
  using KalmanBase = KalmanFilterBase<StateType>;
  //! Standard Filter base type
  using StandardBase = StandardFilterBase<StateType>;

  //! Numeric Scalar Type inherited from base
  using typename KalmanBase::T;

  //! State Type inherited from base
  using typename KalmanBase::State;

  //! Linearized Measurement Model Type
  template <class Measurement, template <class> class CovarianceBase>
  using MeasurementModelType =
      LinearizedMeasurementModel<State, Measurement, CovarianceBase>;

  //! Linearized System Model Type
  template <class Control, template <class> class CovarianceBase>
  using SystemModelType = LinearizedSystemModel<State, Control, CovarianceBase>;

protected:
  //! Kalman Gain Matrix Type
  template <class Measurement>
  using KalmanGain = Kalman::KalmanGain<State, Measurement>;

protected:
  //! State Estimate
  using KalmanBase::_x;
  //! State Covariance Matrix
  using StandardBase::_P;

public:
  /**
   * @brief Constructor
   */
  ExtendedKalmanFilter() {
    // Setup state and covariance
    _P.setIdentity();
  }

  /**
   * @brief Perform filter prediction step using system model and no control
   * input (i.e. \f$ u = 0 \f$)
   *
   * @param [in] s The System model
   * @return The updated state estimate
   */
  template <class Control, template <class> class CovarianceBase>
  const State &predict(SystemModelType<Control, CovarianceBase> &s) {
    // predict state (without control)
    Control u;
    u.setZero();
    return predict(s, u);
  }

  /**
   * @brief Perform filter prediction step using control input \f$u\f$ and
   * corresponding system model
   *
   * @param [in] s The System model
   * @param [in] u The Control input vector
   * @return The updated state estimate
   */
  template <class Control, template <class> class CovarianceBase>
  const State &predict(SystemModelType<Control, CovarianceBase> &s,
                       const Control &u) {
    s.update_jacobians(_x, u);

    // predict state
    _x = s.f(_x, u);

    // predict covariance
    _P = (s._F * _P * s._F.transpose()) +
         (s._W * s.get_covariance() * s._W.transpose());

    // return state prediction
    return this->get_state();
  }

  /**
   * @brief Perform filter update step using measurement \f$z\f$ and
   * corresponding measurement model
   *
   * @param [in] m The Measurement model
   * @param [in] z The measurement vector
   * @return The updated state estimate
   */
  template <class Measurement, template <class> class CovarianceBase>
  const State &update(MeasurementModelType<Measurement, CovarianceBase> &m,
                      const Measurement &z) {
    m.update_jacobians(_x);

    // COMPUTE KALMAN GAIN
    // compute innovation covariance
    Covariance<Measurement> S = (m._H * _P * m._H.transpose()) +
                                (m._V * m.get_covariance() * m._V.transpose());

    // compute kalman gain: K = P H^T S^{-1}
    // For small fixed-size measurement spaces Eigen's closed-form inverse is
    // faster; for large or dynamic sizes LLT solve is more stable and faster.
    KalmanGain<Measurement> K;
    if constexpr (Measurement::RowsAtCompileTime != Eigen::Dynamic &&
                  Measurement::RowsAtCompileTime <= 4) {
      K = _P * m._H.transpose() * S.inverse();
    } else {
      K = S.llt().solve(m._H * _P).transpose();
    }

    // UPDATE STATE ESTIMATE AND COVARIANCE
    // Update state using computed kalman gain and innovation
    _x += K * (z - m.h(_x));

    // Update covariance — eval() prevents aliasing since _P appears on both
    // sides of the compound assignment.
    _P -= (K * m._H * _P).eval();

    // return updated state estimate
    return this->get_state();
  }
};
} // namespace Kalman

#endif
