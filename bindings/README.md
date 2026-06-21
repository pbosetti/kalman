# Language bindings

Python and R bindings expose a **callback-driven Unscented Kalman Filter**: you
supply the state-transition function `f(x)` and the measurement function `h(x)`
in your language of choice, and the C++ library does the filtering. The UKF is
used because it needs no Jacobians, so the binding surface stays tiny.

Both bindings share a small, framework-agnostic core
([`KalmanWrapper.hpp`](KalmanWrapper.hpp)) that erases the library's
compile-time dimensions behind a runtime interface. State and measurement
dimensions from 1 to `KALMAN_BIND_MAX_DIM` (default 6) are supported; fold any
control input into your `f` callback.

## Python (pybind11)

Build the extension:

```sh
pip install pybind11 numpy
cmake -S . -B build -DKALMAN_BUILD_PYTHON=ON \
  -Dpybind11_DIR="$(python -m pybind11 --cmakedir)"
cmake --build build --target kalman_python
```

Use it (a 1-D constant-velocity tracker):

```python
import numpy as np
import kalman  # build/bindings/python on PYTHONPATH

dt = 1.0
f = lambda x: np.array([x[0] + dt * x[1], x[1]])  # state transition
h = lambda x: np.array([x[0]])                     # measure position

ukf = kalman.UnscentedKalmanFilter(state_dim=2, measurement_dim=1, f=f, h=h)
ukf.state = np.array([0.0, 0.0])
ukf.covariance = np.eye(2)
ukf.set_process_noise(1e-3 * np.eye(2))
ukf.set_measurement_noise(np.array([[0.1]]))

ukf.predict()
ukf.update(np.array([2.1]))
print(ukf.state)
```

Tests: `PYTHONPATH=build/bindings/python python -m pytest bindings/python`.

## R (Rcpp)

Build and install the package (`KALMAN_HOME` points at the repository root so the
in-repo headers are found; Eigen comes from `RcppEigen`):

```sh
Rscript -e 'Rcpp::compileAttributes("bindings/r/kalmanr")'
KALMAN_HOME="$(pwd)" R CMD INSTALL bindings/r/kalmanr
```

Use it:

```r
library(kalmanr)

dt <- 1
f <- function(x) c(x[1] + dt * x[2], x[2])  # state transition
h <- function(x) x[1]                        # measure position

ukf <- UnscentedKalmanFilter(2, 1, f, h)
ukf$set_state(c(0, 0))
ukf$set_covariance(diag(2))
ukf$set_process_noise(1e-3 * diag(2))
ukf$set_measurement_noise(matrix(0.1, 1, 1))

ukf$predict()
ukf$update(2.1)
ukf$state()
```

Both bindings are exercised in CI (`.github/workflows/bindings.yml`).
