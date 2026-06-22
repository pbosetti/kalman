#include "TestHelper.h"

#define private public
#define protected public

#include <kalman/SquareRootExtendedKalmanFilter.hpp>

using namespace Kalman;

typedef float T;

TEST(SquareRootExtendedKalmanFilter, init) {
  SquareRootExtendedKalmanFilter<Vector<T, 3>> ekf;
  ASSERT_TRUE(ekf.S.isIdentity());

  SquareRootExtendedKalmanFilter<Matrix<T, 3, 1>> ekfMatrix;
  ASSERT_TRUE(ekfMatrix.S.isIdentity());
}

TEST(SquareRootExtendedKalmanFilter, update_state_covariance) {
  SquareRootExtendedKalmanFilter<Vector<T, 3>> ekf;

  // Setup S_y (innovation covariance square root)
  Matrix<T, 2, 2> Pyy;
  Pyy << 1, 0.1f, 0.1f, 0.5f;
  Cholesky<Matrix<T, 2, 2>> S_y;
  S_y.compute(0.1f * Pyy);
  ASSERT_TRUE(S_y.info() == Eigen::Success);

  // Kalman gain (3-state, 2-measurement)
  Matrix<T, 3, 2> K;
  K << 0.178408514951850f, -0.304105423213381f, -1.110998479472884f,
      0.802838317283324f, -1.246156445345498f, 0.063524243960128f;

  // Start from identity state covariance
  ekf.S.setIdentity();

  // Reference: naive formula P_new = P - K * P_yy * K^T
  Matrix<T, 3, 3> P_ref = ekf.S.reconstructedMatrix() -
                          K * S_y.reconstructedMatrix() * K.transpose();

  // Apply rank-1 downdate
  bool success = ekf.update_state_covariance<Vector<T, 2>>(K, S_y);
  ASSERT_TRUE(success);

  ASSERT_MATRIX_NEAR(P_ref, ekf.S.reconstructedMatrix(), 1e-5f);
}
