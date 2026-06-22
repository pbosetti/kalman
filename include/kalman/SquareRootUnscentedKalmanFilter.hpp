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
#ifndef KALMAN_SQUAREROOTUNSCENTEDKALMANFILTER_HPP_
#define KALMAN_SQUAREROOTUNSCENTEDKALMANFILTER_HPP_

#include "SquareRootFilterBase.hpp"
#include "UnscentedKalmanFilterBase.hpp"

namespace Kalman {

/**
 * @brief Square Root Unscented Kalman Filter (SR-UKF)
 *
 * @note This is the square-root implementation variant of UnscentedKalmanFilter
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
class SquareRootUnscentedKalmanFilter
    : public UnscentedKalmanFilterBase<StateType>,
      public SquareRootFilterBase<StateType> {
public:
  //! Unscented Kalman Filter base type
  using UnscentedBase = UnscentedKalmanFilterBase<StateType>;

  //! Square Root Filter base type
  using SquareRootBase = SquareRootFilterBase<StateType>;

  //! Numeric Scalar Type inherited from base
  using typename UnscentedBase::T;

  //! State Type inherited from base
  using typename UnscentedBase::State;

  //! Measurement Model Type
  template <class Measurement, template <class> class CovarianceBase>
  using MeasurementModelType =
      typename UnscentedBase::template MeasurementModelType<Measurement,
                                                            CovarianceBase>;

  //! System Model Type
  template <class Control, template <class> class CovarianceBase>
  using SystemModelType =
      typename UnscentedBase::template SystemModelType<Control, CovarianceBase>;

protected:
  //! The number of sigma points (depending on state dimensionality)
  using UnscentedBase::SigmaPointCount;

  //! Matrix type containing the sigma state or measurement points
  template <class Type>
  using SigmaPoints = typename UnscentedBase::template SigmaPoints<Type>;

  //! Kalman Gain Matrix Type
  template <class Measurement>
  using KalmanGain = Kalman::KalmanGain<State, Measurement>;

protected:
  // Member variables

  //! State Estimate
  using UnscentedBase::_x;

  //! Square Root of State Covariance
  using SquareRootBase::_S;

  //! Sigma points (state)
  using UnscentedBase::_sigma_state_points;

public:
  /**
   * Constructor
   *
   * See paper for detailed parameter explanation
   *
   * @param alpha Scaling parameter for spread of sigma points (usually \f$ 1E-4
   * \leq \alpha \leq 1 \f$)
   * @param beta Parameter for prior knowledge about the distribution (\f$ \beta
   * = 2 \f$ is optimal for Gaussian)
   * @param kappa Secondary scaling parameter (usually 0)
   */
  SquareRootUnscentedKalmanFilter(T alpha = T(1), T beta = T(2), T kappa = T(0))
      : UnscentedKalmanFilterBase<StateType>(alpha, beta, kappa) {
    // Init covariance to identity
    _S.setIdentity();
  }

  /**
   * @brief Perform filter prediction step using system model and no control
   * input (i.e. \f$ u = 0 \f$)
   *
   * @param [in] s The System model
   * @return The updated state estimate
   */
  template <class Control, template <class> class CovarianceBase>
  const State &predict(const SystemModelType<Control, CovarianceBase> &s) {
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
  const State &predict(const SystemModelType<Control, CovarianceBase> &s,
                       const Control &u) {
    // Compute sigma points
    compute_sigma_points();

    // Compute predicted state
    _x = this->template compute_state_prediction<Control, CovarianceBase>(s, u);

    // Compute predicted covariance
    if (!compute_covariance_square_root_from_sigma_points(
            _x, _sigma_state_points, s.get_covariance_square_root(), _S)) {
      // TODO: handle numerical error
      assert(false);
    }

    // Return predicted state
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
  const State &
  update(const MeasurementModelType<Measurement, CovarianceBase> &m,
         const Measurement &z) {
    SigmaPoints<Measurement> sigma_measurement_points;

    // Predict measurement (and corresponding sigma points)
    Measurement y = this->template compute_measurement_prediction<
        Measurement, CovarianceBase>(m, sigma_measurement_points);

    // Compute square root innovation covariance
    CovarianceSquareRoot<Measurement> S_y;
    if (!compute_covariance_square_root_from_sigma_points(
            y, sigma_measurement_points, m.get_covariance_square_root(), S_y)) {
      // TODO: handle numerical error
      assert(false);
    }

    KalmanGain<Measurement> K;
    compute_kalman_gain(y, sigma_measurement_points, S_y, K);

    // Update state
    _x += K * (z - y);

    // Update state covariance
    if (!update_state_covariance<Measurement>(K, S_y)) {
      // TODO: handle numerical error
      assert(false);
    }

    return this->get_state();
  }

protected:
  /**
   * @brief Compute sigma points from current state estimate and state
   * covariance
   *
   * @note This covers equations (17) and (22) of Algorithm 3.1 in the Paper
   */
  bool compute_sigma_points() {
    // Get square root of covariance
    Matrix<T, State::RowsAtCompileTime, State::RowsAtCompileTime> S =
        _S.matrixL().toDenseMatrix();

    // Set left "block" (first column)
    _sigma_state_points.template leftCols<1>() = _x;
    // Set center block with x + gamma * S
    _sigma_state_points
        .template block<State::RowsAtCompileTime, State::RowsAtCompileTime>(
            0, 1) = (this->_gamma * S).colwise() + _x;
    // Set right block with x - gamma * S
    _sigma_state_points.template rightCols<State::RowsAtCompileTime>() =
        (-this->_gamma * S).colwise() + _x;

    return true;
  }

  /**
   * @brief Compute the Covariance Square root from sigma points and noise
   * covariance
   *
   * @note This covers equations (20) and (21) as well as (25) and (26) of
   * Algorithm 3.1 in the Paper
   *
   * @param [in] mean The mean predicted state or measurement
   * @param [in] sigma_points the predicted sigma state or measurement points
   * @param [in] noise_cov The system or measurement noise covariance (as square
   * root)
   * @param [out] cov The propagated state or innovation covariance (as square
   * root)
   *
   * @return True on success, false if a numerical error is encountered when
   * updating the matrix
   */
  template <class Type>
  bool compute_covariance_square_root_from_sigma_points(
      const Type &mean, const SigmaPoints<Type> &sigma_points,
      const CovarianceSquareRoot<Type> &noise_cov,
      CovarianceSquareRoot<Type> &cov) {
    // Compute QR decomposition of (transposed) augmented matrix
    Matrix<T, 2 * State::RowsAtCompileTime + Type::RowsAtCompileTime,
           Type::RowsAtCompileTime>
        tmp;
    tmp.template topRows<2 * State::RowsAtCompileTime>() =
        std::sqrt(this->_sigma_weights_c[1]) *
        (sigma_points.template rightCols<SigmaPointCount - 1>().colwise() -
         mean)
            .transpose();
    tmp.template bottomRows<Type::RowsAtCompileTime>() =
        noise_cov.matrixU().toDenseMatrix();

    // TODO: Use ColPivHouseholderQR
    Eigen::HouseholderQR<decltype(tmp)> qr(tmp);

    // Set R matrix as upper triangular square root
    cov.setU(qr.matrixQR()
                 .template topRightCorner<Type::RowsAtCompileTime,
                                          Type::RowsAtCompileTime>());

    // Perform additional rank 1 update
    // TODO: According to the paper (Section 3, "Cholesky factor updating") the
    // update
    //       is defined using the square root of the scalar, however the correct
    //       result is obtained when using the weight directly rather than using
    //       the square root It should be checked whether this is a bug in Eigen
    //       or in the Paper
    // T nu = std::copysign( std::sqrt(std::abs(_sigma_weights_c[0])),
    // _sigma_weights_c[0]);
    T nu = this->_sigma_weights_c[0];
    cov.rankUpdate(sigma_points.template leftCols<1>() - mean, nu);

    return (cov.info() == Eigen::Success);
  }

  /**
   * @brief Compute the Kalman Gain from predicted measurement and sigma points
   *and the innovation covariance.
   *
   * @note This covers equations (27) and (28) of Algorithm 3.1 in the Paper
   *
   * We need to solve the equation \f$ K (S_y S_y^T) = P \f$ for \f$ K \f$ using
   *backsubstitution. In order to apply standard backsubstitution using the
   *`solve` method we must bring the equation into the form \f[ AX = B \qquad
   *\text{with } A = S_yS_y^T \f] Thus we transpose the whole equation to obtain
   * \f{align*}{
   *   ( K (S_yS_y^T))^T &= P^T \Leftrightarrow \\
   *   (S_yS_y^T)^T K^T &= P^T \Leftrightarrow \\
   *   (S_yS_y^T) K^T &= P^T
   *\f}
   * Hence our \f$ X := K^T\f$ and \f$ B:= P^T \f$
   *
   * @param [in] y The predicted measurement
   * @param [in] sigma_measurement_points The predicted sigma measurement points
   * @param [in] S_y The innovation covariance as square-root
   * @param [out] K The computed Kalman Gain matrix \f$ K \f$
   */
  template <class Measurement>
  bool
  compute_kalman_gain(const Measurement &y,
                      const SigmaPoints<Measurement> &sigma_measurement_points,
                      const CovarianceSquareRoot<Measurement> &S_y,
                      KalmanGain<Measurement> &K) {
    Matrix<T, State::RowsAtCompileTime, Measurement::RowsAtCompileTime> P =
        (_sigma_state_points.colwise() - _x) *
        this->_sigma_weights_c.asDiagonal() *
        (sigma_measurement_points.colwise() - y).transpose();

    K = S_y.solve(P.transpose()).transpose();
    return true;
  }

  /**
   * @brief Update the state covariance matrix using the Kalman Gain and the
   * Innovation Covariance
   *
   * @note This covers equations (29) and (30) of Algorithm 3.1 in the Paper
   *
   * @param [in] K The computed Kalman Gain matrix
   * @param [in] S_y The innovation covariance as square-root
   * @return True on success, false if a numerical error is encountered when
   * updating the matrix
   */
  template <class Measurement>
  bool update_state_covariance(const KalmanGain<Measurement> &K,
                               const CovarianceSquareRoot<Measurement> &S_y) {
    KalmanGain<Measurement> U = K * S_y.matrixL();
    for (int i = 0; i < U.cols(); ++i) {
      _S.rankUpdate(U.col(i), -1);

      if (_S.info() == Eigen::NumericalIssue) {
        // TODO: handle numerical issues in some sensible way
        return false;
      }
    }

    return true;
  }
};
} // namespace Kalman

#endif
