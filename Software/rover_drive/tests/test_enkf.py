import math

import numpy as np
import pytest

from rover_sim.control import CommandBus, DirectionMode, FixType, RoverState

from rover_drive.estimation import (
    STATE_HEADING,
    STATE_SPEED,
    STATE_X,
    STATE_Y,
    STATE_YAW_RATE,
    EnKFConfig,
    EnsembleKalmanFilter,
)


def _x0_origin() -> np.ndarray:
    return np.zeros(5)


# ---------- initialization ----------


def test_initialize_sets_ensemble_mean_near_x0():
    f = EnsembleKalmanFilter(EnKFConfig(ensemble_size=512, seed=42))
    x0 = np.array([10.0, 20.0, 0.5, 1.0, 0.1])
    f.initialize(x0)

    assert f.initialized
    assert np.linalg.norm(f.mean - x0) < 0.1


def test_initialize_rejects_wrong_shape():
    f = EnsembleKalmanFilter()
    with pytest.raises(ValueError):
        f.initialize(np.zeros(3))


def test_reset_clears_initialized_flag():
    f = EnsembleKalmanFilter(EnKFConfig(seed=1))
    f.initialize(_x0_origin())
    assert f.initialized
    f.reset()
    assert not f.initialized


def test_predict_before_initialize_raises():
    f = EnsembleKalmanFilter()
    with pytest.raises(RuntimeError):
        f.predict(cmd=None, dt=0.1)


def test_update_before_initialize_raises():
    f = EnsembleKalmanFilter()
    with pytest.raises(RuntimeError):
        f.update(np.zeros(2), obs_noise_std=0.1)


# ---------- predict ----------


def test_predict_on_stationary_rover_grows_position_variance():
    f = EnsembleKalmanFilter(EnKFConfig(ensemble_size=200, seed=7))
    f.initialize(_x0_origin())
    sx0, sy0 = f.position_std()

    for _ in range(20):
        f.predict(cmd=None, dt=0.1)

    sx1, sy1 = f.position_std()
    assert sx1 > sx0
    assert sy1 > sy0


def test_predict_with_dt_zero_is_noop():
    f = EnsembleKalmanFilter(EnKFConfig(seed=3))
    f.initialize(_x0_origin())
    before = f.ensemble.copy()
    f.predict(cmd=None, dt=0.0)
    np.testing.assert_array_equal(f.ensemble, before)


# ---------- update ----------


def test_update_reduces_position_variance():
    f = EnsembleKalmanFilter(EnKFConfig(ensemble_size=200, seed=11))
    f.initialize(_x0_origin())
    sx0, sy0 = f.position_std()

    f.update(observation=np.array([0.0, 0.0]), obs_noise_std=0.01)
    sx1, sy1 = f.position_std()

    assert sx1 < sx0
    assert sy1 < sy0


def test_low_noise_update_snaps_mean_to_observation():
    f = EnsembleKalmanFilter(EnKFConfig(ensemble_size=200, seed=13))
    f.initialize(_x0_origin())

    f.update(observation=np.array([3.0, -2.0]), obs_noise_std=0.001)
    mean_xy = f.mean[:2]
    assert mean_xy[0] == pytest.approx(3.0, abs=0.1)
    assert mean_xy[1] == pytest.approx(-2.0, abs=0.1)


def test_high_noise_update_barely_moves_mean():
    f = EnsembleKalmanFilter(EnKFConfig(ensemble_size=200, seed=17))
    f.initialize(_x0_origin())
    mean0 = f.mean[:2].copy()

    # Distant observation with huge uncertainty; Kalman gain should be small
    f.update(observation=np.array([100.0, 100.0]), obs_noise_std=1000.0)
    mean1 = f.mean[:2].copy()
    assert np.linalg.norm(mean1 - mean0) < 1.0


def test_zero_noise_observation_collapses_position_to_measurement():
    f = EnsembleKalmanFilter(EnKFConfig(ensemble_size=64, seed=19))
    f.initialize(_x0_origin())
    f.update(observation=np.array([5.0, 7.0]), obs_noise_std=0.0)

    assert np.all(f.ensemble[:, STATE_X] == 5.0)
    assert np.all(f.ensemble[:, STATE_Y] == 7.0)


def test_update_rejects_wrong_observation_shape():
    f = EnsembleKalmanFilter()
    f.initialize(_x0_origin())
    with pytest.raises(ValueError):
        f.update(observation=np.zeros(3), obs_noise_std=1.0)


# ---------- end-to-end convergence ----------


def test_converges_to_truth_from_biased_initial():
    """A stationary rover observed with realistic FIXED-quality noise should
    converge the filter mean to the truth within a few tens of samples."""
    f = EnsembleKalmanFilter(EnKFConfig(ensemble_size=128, seed=42))
    # Wrong initial position by 5 m in x, 5 m in y
    f.initialize(np.array([5.0, 5.0, 0.0, 0.0, 0.0]))

    rng = np.random.default_rng(0)
    truth = np.zeros(2)
    for _ in range(100):
        f.predict(cmd=None, dt=0.1)
        noisy_obs = truth + rng.normal(0, 0.02, size=2)
        f.update(noisy_obs, obs_noise_std=0.02)

    assert np.linalg.norm(f.mean[:2] - truth) < 0.1


def test_degraded_fix_shows_larger_posterior_variance():
    """The same observation vector fused with SPP-scale noise should leave
    wider ensemble spread than a FIXED-scale fuse, matching the physical
    expectation that low-quality GNSS constrains the state less."""
    f_fixed = EnsembleKalmanFilter(EnKFConfig(ensemble_size=200, seed=23))
    f_fixed.initialize(_x0_origin())
    f_fixed.update(np.array([0.0, 0.0]), obs_noise_std=0.015)  # FIXED envelope

    f_spp = EnsembleKalmanFilter(EnKFConfig(ensemble_size=200, seed=23))
    f_spp.initialize(_x0_origin())
    f_spp.update(np.array([0.0, 0.0]), obs_noise_std=1.5)  # SPP envelope

    sx_fixed, _ = f_fixed.position_std()
    sx_spp, _ = f_spp.position_std()
    assert sx_spp > sx_fixed * 5


# ---------- integration with RoverState ----------


def test_update_from_gnss_applies_when_fix_is_valid():
    f = EnsembleKalmanFilter(EnKFConfig(ensemble_size=200, seed=29))
    f.initialize(_x0_origin())
    state = RoverState(
        t=0.0, x=1.0, y=2.0, fix_type=FixType.FIXED, position_std_m=0.01
    )
    assert f.update_from_gnss(state) is True
    assert f.mean[STATE_X] == pytest.approx(1.0, abs=0.2)
    assert f.mean[STATE_Y] == pytest.approx(2.0, abs=0.2)


def test_update_from_gnss_skips_when_fix_is_none():
    f = EnsembleKalmanFilter(EnKFConfig(ensemble_size=64, seed=31))
    f.initialize(_x0_origin())
    before = f.mean.copy()

    state = RoverState(t=0.0, x=99.0, y=99.0, fix_type=FixType.NONE)
    assert f.update_from_gnss(state) is False
    np.testing.assert_array_equal(f.mean, before)


def test_predict_with_command_follows_expected_trajectory():
    """With a constant forward throttle and closed-loop zero steer, the filter
    mean should advance eastward consistent with the commanded speed, even
    starting from a stopped state."""
    f = EnsembleKalmanFilter(EnKFConfig(ensemble_size=200, seed=37))
    f.initialize(_x0_origin())

    cmd = CommandBus(
        throttle=0.5, steer=0.0, direction_mode=DirectionMode.CLOSED_LOOP
    )
    rng = np.random.default_rng(0)
    for step in range(60):
        f.predict(cmd=cmd, dt=0.1)
        # Occasional noisy observation at the expected progressed position.
        # Mean speed = 0.5 * 5.56 = 2.78 m/s; after 6 s ≈ 16.68 m east.
        if step > 5:
            truth_x = 0.5 * 5.56 * (step * 0.1 - 0.2)  # rough first-order estimate
            noisy = np.array([truth_x, 0.0]) + rng.normal(0, 0.02, 2)
            f.update(noisy, obs_noise_std=0.02)

    assert f.mean[STATE_X] > 10.0
    assert f.mean[STATE_SPEED] > 1.0
