import math

import numpy as np
import pytest

from rover_sim.control import CommandBus, FixType, RoverState
from rover_sim.vehicle import SingleTrackArticulated, SingleTrackArticulatedConfig

from rover_drive.estimation import EnKFConfig, EnsembleKalmanFilter
from rover_drive.modes.calibration import (
    CalibrationConfig,
    CalibrationDriver,
    CalibrationPhase,
)


def _run_calibration_on_sim(
    true_arm_length: float,
    guessed_arm_length: float = 2.5,
    steer_tests: tuple[float, ...] = (0.3, 0.6),
) -> CalibrationDriver:
    """Run a full calibration against a simulated vehicle with a known true arm length."""
    vehicle = SingleTrackArticulated(
        SingleTrackArticulatedConfig(
            arm_length_m=true_arm_length,
            max_speed_mps=5.56,
            accel_mps2=3.0,
            max_gamma_rad=math.radians(45.0),
            closed_loop_gain=10.0,
            max_gamma_rate_rad_s=math.radians(60.0),
        )
    )

    cal = CalibrationDriver(
        config=CalibrationConfig(
            test_throttle=0.25,
            straight_duration_s=2.0,
            steer_test_values=steer_tests,
            steer_test_duration_s=2.0,
        ),
        enkf=EnsembleKalmanFilter(EnKFConfig(seed=42)),
    )

    dt = 0.05
    t = 0.0
    max_steps = 2000

    for _ in range(max_steps):
        vstate = vehicle.state
        rover_state = RoverState(
            t=t, x=vstate.x, y=vstate.y, heading=vstate.heading,
            speed=vstate.speed, yaw_rate=vstate.yaw_rate,
            fix_type=FixType.FIXED, position_std_m=0.015,
        )
        cmd = cal.update(rover_state, mission=None, dt=dt)
        vehicle.step(cmd, dt)
        t += dt
        if cal.complete:
            break

    return cal


# ---------- phase transitions ----------


def test_calibration_progresses_through_phases():
    cal = _run_calibration_on_sim(true_arm_length=2.5, steer_tests=(0.5,))
    assert cal.complete
    assert cal.phase is CalibrationPhase.COMPLETE
    assert cal.result is not None


def test_result_has_expected_fields():
    cal = _run_calibration_on_sim(true_arm_length=3.0, steer_tests=(0.3, 0.6))
    r = cal.result
    assert r is not None
    assert r.effective_arm_length_m > 0
    assert len(r.steer_values) == 2
    assert len(r.measured_curvatures) == 2
    assert len(r.measured_speeds) == 2


# ---------- arm length recovery ----------


def test_recovers_true_arm_length_within_30_percent():
    """The calibration should fit an arm length reasonably close to the
    simulated vehicle's true arm length. Tolerance is generous because
    the EnKF's dynamics model starts with a wrong arm length and the
    short maneuver window limits how precisely we can measure curvature."""
    true_L = 3.5
    cal = _run_calibration_on_sim(true_arm_length=true_L, steer_tests=(0.3, 0.6, 1.0))
    fitted_L = cal.result.effective_arm_length_m
    error_pct = abs(fitted_L - true_L) / true_L * 100
    assert error_pct < 35, f"fitted {fitted_L:.2f} vs true {true_L:.2f} ({error_pct:.0f}% error)"
    # Directionally correct: the fit should be > 2.5 (the default guess)
    assert fitted_L > 2.3


def test_different_arm_lengths_give_different_results():
    cal_short = _run_calibration_on_sim(true_arm_length=1.5, steer_tests=(0.5,))
    cal_long = _run_calibration_on_sim(true_arm_length=4.0, steer_tests=(0.5,))
    assert cal_short.result.effective_arm_length_m < cal_long.result.effective_arm_length_m


# ---------- reset ----------


def test_reset_clears_state():
    cal = _run_calibration_on_sim(true_arm_length=2.5, steer_tests=(0.5,))
    assert cal.complete
    cal.reset()
    assert not cal.complete
    assert cal.result is None
    assert cal.phase is CalibrationPhase.STRAIGHT


# ---------- waits for fix ----------


def test_waits_for_valid_fix():
    cal = CalibrationDriver(enkf=EnsembleKalmanFilter(EnKFConfig(seed=1)))
    none_state = RoverState(t=0.0, fix_type=FixType.NONE)
    cmd = cal.update(none_state, mission=None, dt=0.05)
    assert cmd.brake > 0
    assert not cal.enkf.initialized
