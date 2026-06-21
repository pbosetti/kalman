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

// R bindings (Rcpp) for the Unscented Kalman Filter. The user supplies the
// state-transition function f(x) and the measurement function h(x) as ordinary
// R functions operating on numeric vectors.

// [[Rcpp::depends(RcppEigen)]]
#include <RcppEigen.h>

#include "KalmanWrapper.hpp"

using Kalman::Bindings::Filter;

namespace {

// Wrap an R function as an Eigen-vector-valued callback. The R function is kept
// alive by being stored in the XPtr's protection slot (see kalman_make).
Kalman::Bindings::TransitionFn as_callback(Rcpp::Function fn) {
  return [fn](const Eigen::VectorXd &x) -> Eigen::VectorXd {
    return Rcpp::as<Eigen::VectorXd>(fn(Rcpp::wrap(x)));
  };
}

Filter &deref(SEXP handle) {
  Rcpp::XPtr<Filter> ptr(handle);
  if (!ptr)
    Rcpp::stop("invalid Kalman filter handle");
  return *ptr;
}

} // namespace

// [[Rcpp::export]]
SEXP kalman_make(int state_dim, int measurement_dim, Rcpp::Function f,
                 Rcpp::Function h) {
  std::unique_ptr<Filter> filter = Kalman::Bindings::make_unscented(
      state_dim, measurement_dim, as_callback(f), as_callback(h));
  // Protect the R callbacks for as long as the handle is alive.
  Rcpp::List prot = Rcpp::List::create(f, h);
  return Rcpp::XPtr<Filter>(filter.release(), true, R_NilValue, prot);
}

// [[Rcpp::export]]
void kalman_set_state(SEXP handle, Eigen::VectorXd x) {
  deref(handle).set_state(x);
}

// [[Rcpp::export]]
Eigen::VectorXd kalman_state(SEXP handle) { return deref(handle).state(); }

// [[Rcpp::export]]
void kalman_set_covariance(SEXP handle, Eigen::MatrixXd P) {
  deref(handle).set_covariance(P);
}

// [[Rcpp::export]]
Eigen::MatrixXd kalman_covariance(SEXP handle) {
  return deref(handle).covariance();
}

// [[Rcpp::export]]
void kalman_set_process_noise(SEXP handle, Eigen::MatrixXd Q) {
  deref(handle).set_process_noise(Q);
}

// [[Rcpp::export]]
void kalman_set_measurement_noise(SEXP handle, Eigen::MatrixXd R) {
  deref(handle).set_measurement_noise(R);
}

// [[Rcpp::export]]
Eigen::VectorXd kalman_predict(SEXP handle) { return deref(handle).predict(); }

// [[Rcpp::export]]
Eigen::VectorXd kalman_update(SEXP handle, Eigen::VectorXd z) {
  return deref(handle).update(z);
}
