# Smoke tests for the R bindings, mirroring the Python test-suite.

make_cv_filter <- function(dt = 1) {
  f <- function(x) c(x[1] + dt * x[2], x[2])
  h <- function(x) x[1]
  ukf <- UnscentedKalmanFilter(2, 1, f, h)
  ukf$set_state(c(0, 0))
  ukf$set_covariance(diag(2))
  ukf$set_process_noise(1e-3 * diag(2))
  ukf$set_measurement_noise(matrix(0.1, 1, 1))
  ukf
}

test_that("metadata is exposed", {
  ukf <- make_cv_filter()
  expect_equal(ukf$state_dim, 2L)
  expect_equal(ukf$measurement_dim, 1L)
})

test_that("predict advances position by velocity", {
  ukf <- make_cv_filter(dt = 1)
  ukf$set_state(c(0, 2))
  x <- ukf$predict()
  expect_equal(x[1], 2, tolerance = 1e-6)
  expect_equal(x[2], 2, tolerance = 1e-6)
})

test_that("filter converges to the true velocity", {
  ukf <- make_cv_filter(dt = 1)
  set.seed(0)
  true_v <- 2
  pos <- 0
  for (i in 1:60) {
    pos <- pos + true_v
    ukf$predict()
    ukf$update(pos + rnorm(1, 0, 0.3))
  }
  est <- ukf$state()
  expect_equal(est[2], true_v, tolerance = 0.3)
  expect_true(all(is.finite(ukf$covariance())))
})

test_that("an out-of-range dimension is rejected", {
  expect_error(UnscentedKalmanFilter(99, 1, function(x) x, function(x) x[1]))
})
