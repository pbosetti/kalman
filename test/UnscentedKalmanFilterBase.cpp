#include "models/Quadratic.hpp"
#include "TestHelper.h"

#define private public
#define protected public

#include <kalman/UnscentedKalmanFilterBase.hpp>

using namespace Kalman;

template <class StateType>
class ConcreteUKF : public UnscentedKalmanFilterBase<StateType> {
public:
  typedef UnscentedKalmanFilterBase<StateType> Base;
  using typename Base::T;

  ConcreteUKF(T alpha, T beta, T kappa) : Base(alpha, beta, kappa) {}
};

typedef float T;

TEST(UnscentedKalmanFilterBase, init) {
  auto ukf = ConcreteUKF<Vector<T, 3>>(1, 2, 1);

  // x should be zero
  ASSERT_FLOAT_EQ(0, ukf._x[0]);
  ASSERT_FLOAT_EQ(0, ukf._x[1]);
  ASSERT_FLOAT_EQ(0, ukf._x[2]);
}

TEST(UnscentedKalmanFilterBase, computeSigmaWeights) {
  T alpha = 1, beta = 2, kappa = 1;

  auto ukf = ConcreteUKF<Vector<T, 3>>(alpha, beta, kappa);
  ukf.compute_weights();

  ASSERT_FLOAT_EQ(2, ukf._gamma);
  ASSERT_FLOAT_EQ(1, ukf._lambda);

  ASSERT_FLOAT_EQ(0.25, ukf._sigma_weights_m[0]);
  ASSERT_FLOAT_EQ(2.25, ukf._sigma_weights_c[0]);

  for (size_t i = 1; i < 7; i++) {
    ASSERT_FLOAT_EQ(0.125, ukf._sigma_weights_c[i]);
    ASSERT_FLOAT_EQ(0.125, ukf._sigma_weights_m[i]);
  }
}

TEST(UnscentedKalmanFilterBase, compute_sigma_point_transition) {
  T alpha = 1, beta = 2, kappa = 1;

  auto ukf = ConcreteUKF<Vector<T, 3>>(alpha, beta, kappa);
  auto model = Kalman::Test::Models::QuadraticSystemModel<Vector<T, 3>>();

  // Init variables
  ukf._sigma_state_points << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
      16, 17, 18, 19, 20, 21;

  // Control vector
  Vector<T, 3> u;
  u << 1, 2, 3;

  // Compute reference result
  Matrix<T, 3, 7> ref =
      (ukf._sigma_state_points.cwiseProduct(ukf._sigma_state_points).colwise() +
       u)
          .eval();

  // Compute transition
  ukf.compute_sigma_point_transition(model, u);

  ASSERT_MATRIX_NEAR(ref, ukf._sigma_state_points, 1e-10);
}

TEST(UnscentedKalmanFilterBase, compute_sigma_point_measurements) {
  T alpha = 1, beta = 2, kappa = 1;

  auto ukf = ConcreteUKF<Vector<T, 3>>(alpha, beta, kappa);
  auto model = Kalman::Test::Models::QuadraticMeasurementModel<Vector<T, 3>,
                                                               Vector<T, 2>>();

  // Init variables
  ukf._sigma_state_points << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
      16, 17, 18, 19, 20, 21;

  // Compute Reference result
  Matrix<T, 2, 7> tmp = ukf._sigma_state_points.template topRows<2>();
  Matrix<T, 2, 7> ref = tmp.cwiseProduct(tmp).eval();

  typename ConcreteUKF<Vector<T, 3>>::template SigmaPoints<Vector<T, 2>> points;

  ukf.compute_sigma_point_measurements(model, points);

  // Compare ref and result
  ASSERT_MATRIX_NEAR(ref, points, 1e-10);
}

TEST(UnscentedKalmanFilterBase, compute_prediction_from_sigma_points) {
  T alpha = 1, beta = 2, kappa = 1;

  auto ukf = ConcreteUKF<Vector<T, 3>>(alpha, beta, kappa);

  // Init variables
  ukf._sigma_weights_m << 7, 6, 5, 4, 3, 2, 1;

  typename ConcreteUKF<Vector<T, 3>>::SigmaPoints<Vector<T, 4>> points;
  points << 1, 2, 3, 4, 5, 6, 7, 10, 20, 30, 40, 50, 60, 70, 100, 200, 300, 400,
      500, 600, 700, 1000, 2000, 3000, 4000, 5000, 6000, 7000;

  Vector<T, 4> x =
      ukf.compute_prediction_from_sigma_points<Vector<T, 4>>(points);

  ASSERT_FLOAT_EQ(84, x[0]);
  ASSERT_FLOAT_EQ(840, x[1]);
  ASSERT_FLOAT_EQ(8400, x[2]);
  ASSERT_FLOAT_EQ(84000, x[3]);
}
