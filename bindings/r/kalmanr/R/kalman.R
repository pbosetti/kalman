# R user-facing API for the Kalman filter bindings.

#' Create an Unscented Kalman Filter
#'
#' Builds a callback-driven Unscented Kalman Filter. Supply the
#' state-transition function \code{f(x)} and the measurement function
#' \code{h(x)} as ordinary R functions operating on numeric vectors. Fold any
#' control input into \code{f}.
#'
#' @param state_dim Integer number of state variables (1..6).
#' @param measurement_dim Integer number of measured variables (1..6).
#' @param f Function mapping a state vector to the next state vector.
#' @param h Function mapping a state vector to an expected measurement vector.
#'
#' @return An object of class \code{UnscentedKalmanFilter}: a list of methods
#'   \code{set_state(x)}, \code{state()}, \code{set_covariance(P)},
#'   \code{covariance()}, \code{set_process_noise(Q)},
#'   \code{set_measurement_noise(R)}, \code{predict()} and \code{update(z)}.
#'
#' @examples
#' \dontrun{
#' dt <- 1
#' f <- function(x) c(x[1] + dt * x[2], x[2])
#' h <- function(x) x[1]
#' ukf <- UnscentedKalmanFilter(2, 1, f, h)
#' ukf$set_state(c(0, 0))
#' ukf$set_covariance(diag(2))
#' ukf$set_process_noise(1e-3 * diag(2))
#' ukf$set_measurement_noise(matrix(0.1, 1, 1))
#' ukf$predict()
#' ukf$update(2.1)
#' ukf$state()
#' }
#'
#' @export
UnscentedKalmanFilter <- function(state_dim, measurement_dim, f, h) {
  handle <- kalman_make(as.integer(state_dim), as.integer(measurement_dim),
                        f, h)

  self <- list(
    state_dim = as.integer(state_dim),
    measurement_dim = as.integer(measurement_dim),
    set_state = function(x) invisible(kalman_set_state(handle, as.numeric(x))),
    state = function() kalman_state(handle),
    set_covariance = function(P) invisible(kalman_set_covariance(handle, as.matrix(P))),
    covariance = function() kalman_covariance(handle),
    set_process_noise = function(Q) invisible(kalman_set_process_noise(handle, as.matrix(Q))),
    set_measurement_noise = function(R) invisible(kalman_set_measurement_noise(handle, as.matrix(R))),
    predict = function() kalman_predict(handle),
    update = function(z) kalman_update(handle, as.numeric(z))
  )
  class(self) <- "UnscentedKalmanFilter"
  self
}
