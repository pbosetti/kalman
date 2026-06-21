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
#ifndef KALMAN_UNSCENTEDKALMANFILTERBASE_HPP_
#define KALMAN_UNSCENTEDKALMANFILTERBASE_HPP_

#include <cassert>

#include "KalmanFilterBase.hpp"
#include "MeasurementModel.hpp"
#include "SystemModel.hpp"

namespace Kalman {

/**
 * @brief Abstract Base for Unscented Kalman Filters
 *
 * This implementation is based upon [The square-root unscented Kalman filter
 * for state and
 * parameter-estimation](http://dx.doi.org/10.1109/ICASSP.2001.940586) by
 * Rudolph van der Merwe and Eric A. Wan. Whenever "the paper" is referenced
 * within this file then this paper is meant.
 *
 * @param StateType The vector-type of the system state (usually some type
 * derived from Kalman::Vector)
 */
template <class StateType>
class UnscentedKalmanFilterBase : public KalmanFilterBase<StateType> {
public:
  //! Kalman Filter base type
  using Base = KalmanFilterBase<StateType>;

  //! Numeric Scalar Type inherited from base
  using typename Base::T;

  //! State Type inherited from base
  using typename Base::State;

  //! Measurement Model Type
  template <class Measurement, template <class> class CovarianceBase>
  using MeasurementModelType =
      MeasurementModel<State, Measurement, CovarianceBase>;

  //! System Model Type
  template <class Control, template <class> class CovarianceBase>
  using SystemModelType = SystemModel<State, Control, CovarianceBase>;

protected:
  //! The number of sigma points (depending on state dimensionality)
  static constexpr int SigmaPointCount = 2 * State::RowsAtCompileTime + 1;

  //! Vector containg the sigma scaling weights
  using SigmaWeights = Vector<T, SigmaPointCount>;

  //! Matrix type containing the sigma state or measurement points
  template <class Type>
  using SigmaPoints = Matrix<T, Type::RowsAtCompileTime, SigmaPointCount>;

protected:
  // Member variables

  //! State Estimate
  using Base::_x;

  //! Sigma weights (m)
  SigmaWeights _sigma_weights_m;
  //! Sigma weights (c)
  SigmaWeights _sigma_weights_c;

  //! Sigma points (state)
  SigmaPoints<State> _sigma_state_points;

  // Weight parameters
  T _alpha; //!< Scaling parameter for spread of sigma points (usually \f$ 1E-4
            //!< \leq \alpha \leq 1 \f$)
  T _beta;  //!< Parameter for prior knowledge about the distribution (\f$ \beta
            //!< = 2 \f$ is optimal for Gaussian)
  T _kappa; //!< Secondary scaling parameter (usually 0)
  T _gamma; //!< \f$ \gamma = \sqrt{L + \lambda} \f$ with \f$ L \f$ being the
            //!< state dimensionality
  T _lambda; //!< \f$ \lambda = \alpha^2 ( L + \kappa ) - L\f$ with \f$ L \f$
             //!< being the state dimensionality

protected:
  /**
   * Constructor
   *
   * See paper for parameter explanation
   *
   * @param alpha Scaling parameter for spread of sigma points (usually 1e-4 <=
   * alpha <= 1)
   * @param beta Parameter for prior knowledge about the distribution (beta = 2
   * is optimal for Gaussian)
   * @param kappa Secondary scaling parameter (usually 0)
   */
  UnscentedKalmanFilterBase(T alpha = T(1), T beta = T(2), T kappa = T(0))
      : _alpha(alpha), _beta(beta), _kappa(kappa) {
    // Pre-compute all weights
    compute_weights();

    // Setup state and covariance
    _x.setZero();
  }

  /**
   * @brief Compute predicted state using system model and control input
   *
   * @param [in] s The System Model
   * @param [in] u The Control input
   * @return The predicted state
   */
  template <class Control, template <class> class CovarianceBase>
  State
  compute_state_prediction(const SystemModelType<Control, CovarianceBase> &s,
                           const Control &u) {
    // Pass each sigma point through non-linear state transition function
    compute_sigma_point_transition(s, u);

    // Compute predicted state from predicted sigma points
    return compute_prediction_from_sigma_points<State>(_sigma_state_points);
  }

  /**
   * @brief Compute predicted measurement using measurement model and predicted
   * sigma measurements
   *
   * @param [in] m The Measurement Model
   * @param [in] sigma_measurement_points The predicted sigma measurement points
   * @return The predicted measurement
   */
  template <class Measurement, template <class> class CovarianceBase>
  Measurement compute_measurement_prediction(
      const MeasurementModelType<Measurement, CovarianceBase> &m,
      SigmaPoints<Measurement> &sigma_measurement_points) {
    // Predict measurements for each sigma point
    compute_sigma_point_measurements<Measurement>(m, sigma_measurement_points);

    // Predict measurement from sigma measurement points
    return compute_prediction_from_sigma_points<Measurement>(
        sigma_measurement_points);
  }

  /**
   * @brief Compute sigma weights
   */
  void compute_weights() {
    T L = T(State::RowsAtCompileTime);
    _lambda = _alpha * _alpha * (L + _kappa) - L;
    _gamma = std::sqrt(L + _lambda);

    // Make sure L != -lambda to avoid division by zero
    assert(std::abs(L + _lambda) > 1e-6);

    // Make sure L != -kappa to avoid division by zero
    assert(std::abs(L + _kappa) > 1e-6);

    T W_m_0 = _lambda / (L + _lambda);
    T W_c_0 = W_m_0 + (T(1) - _alpha * _alpha + _beta);
    T W_i = T(1) / (T(2) * _alpha * _alpha * (L + _kappa));

    // Make sure W_i > 0 to avoid square-root of negative number
    assert(W_i > T(0));

    _sigma_weights_m[0] = W_m_0;
    _sigma_weights_c[0] = W_c_0;

    for (int i = 1; i < SigmaPointCount; ++i) {
      _sigma_weights_m[i] = W_i;
      _sigma_weights_c[i] = W_i;
    }
  }

  /**
   * @brief Predict expected sigma states from current sigma states using system
   * model and control input
   *
   * @note This covers equation (18) of Algorithm 3.1 in the Paper
   *
   * @param [in] s The System Model
   * @param [in] u The Control input
   */
  template <class Control, template <class> class CovarianceBase>
  void compute_sigma_point_transition(
      const SystemModelType<Control, CovarianceBase> &s, const Control &u) {
    for (int i = 0; i < SigmaPointCount; ++i) {
      _sigma_state_points.col(i) = s.f(_sigma_state_points.col(i), u);
    }
  }

  /**
   * @brief Predict the expected sigma measurements from predicted sigma states
   * using measurement model
   *
   * @note This covers equation (23) of Algorithm 3.1 in the Paper
   *
   * @param [in] m The Measurement model
   * @param [out] sigma_measurement_points The struct of expected sigma
   * measurements to be computed
   */
  template <class Measurement, template <class> class CovarianceBase>
  void compute_sigma_point_measurements(
      const MeasurementModelType<Measurement, CovarianceBase> &m,
      SigmaPoints<Measurement> &sigma_measurement_points) {
    for (int i = 0; i < SigmaPointCount; ++i) {
      sigma_measurement_points.col(i) = m.h(_sigma_state_points.col(i));
    }
  }

  /**
   * @brief Compute state or measurement prediciton from sigma points using
   * pre-computed sigma weights
   *
   * @note This covers equations (19) and (24) of Algorithm 3.1 in the Paper
   *
   * @param [in] sigma_points The computed sigma points of the desired type
   * (state or measurement)
   * @return The prediction
   */
  template <class Type>
  Type
  compute_prediction_from_sigma_points(const SigmaPoints<Type> &sigma_points) {
    // Use efficient matrix x vector computation
    return sigma_points * _sigma_weights_m;
  }
};
} // namespace Kalman

#endif
