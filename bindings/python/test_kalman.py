"""Smoke tests for the Python bindings of the Kalman filter library.

Run after building the extension (so that ``kalman`` is importable), e.g.:

    cmake -S . -B build -DKALMAN_BUILD_PYTHON=ON
    cmake --build build --target kalman_python
    PYTHONPATH=build/bindings/python python -m pytest bindings/python
"""

import numpy as np
import pytest

import kalman


def make_cv_filter(dt=1.0):
    """1-D constant-velocity tracker: state = [position, velocity]."""

    def f(x):
        return np.array([x[0] + dt * x[1], x[1]])

    def h(x):
        return np.array([x[0]])

    ukf = kalman.UnscentedKalmanFilter(state_dim=2, measurement_dim=1, f=f, h=h)
    ukf.state = np.array([0.0, 0.0])
    ukf.covariance = np.eye(2)
    ukf.set_process_noise(1e-3 * np.eye(2))
    ukf.set_measurement_noise(np.array([[0.1]]))
    return ukf


def test_dims_and_metadata():
    ukf = make_cv_filter()
    assert ukf.state_dim == 2
    assert ukf.measurement_dim == 1
    assert kalman.MAX_DIM >= 2


def test_predict_advances_position_by_velocity():
    ukf = make_cv_filter(dt=1.0)
    ukf.state = np.array([0.0, 2.0])
    x = ukf.predict()
    assert x[0] == pytest.approx(2.0, abs=1e-6)
    assert x[1] == pytest.approx(2.0, abs=1e-6)


def test_filter_converges_to_true_velocity():
    ukf = make_cv_filter(dt=1.0)
    true_v = 2.0
    rng = np.random.default_rng(0)
    pos = 0.0
    for _ in range(60):
        pos += true_v
        ukf.predict()
        z = np.array([pos + rng.normal(0.0, 0.3)])
        ukf.update(z)
    est = ukf.state
    assert est[1] == pytest.approx(true_v, abs=0.3)
    # Covariance must stay symmetric positive (diagonal) and finite.
    cov = ukf.covariance
    assert np.all(np.isfinite(cov))
    assert np.all(np.diag(cov) > 0.0)


def test_invalid_dimension_raises():
    with pytest.raises(Exception):
        kalman.UnscentedKalmanFilter(
            state_dim=kalman.MAX_DIM + 1,
            measurement_dim=1,
            f=lambda x: x,
            h=lambda x: x[:1],
        )


def test_wrong_measurement_size_raises():
    ukf = make_cv_filter()
    with pytest.raises(Exception):
        ukf.update(np.array([1.0, 2.0]))  # measurement_dim is 1


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
