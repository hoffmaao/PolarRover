"""Pre-mission calibration routine — measures steering response before autonomous navigation."""

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

import numpy as np

from rover_sim.control import CommandBus, DirectionMode, FixType, RoverState
from rover_sim.missions import Mission

from rover_drive.estimation import (
    STATE_HEADING,
    STATE_SPEED,
    STATE_X,
    STATE_Y,
    STATE_YAW_RATE,
    EnsembleKalmanFilter,
)


class CalibrationPhase(Enum):
    STRAIGHT = "straight"      # drive straight to establish baseline speed
    STEER_TEST = "steer_test"  # apply known steer, measure curvature
    FITTING = "fitting"        # all maneuvers done, fit the model
    COMPLETE = "complete"      # calibration done, results available


@dataclass
class CalibrationConfig:
    """
    Configuration for the pre-mission calibration routine.

    The rover executes a short scripted maneuver:
      1. Drive straight for ``straight_duration_s`` at ``test_throttle``
         to establish baseline speed.
      2. For each value in ``steer_test_values``, hold that steer command
         for ``steer_test_duration_s`` and measure the resulting yaw rate
         from the EnKF to compute curvature = yaw_rate / speed.
      3. Fit ``effective_arm_length_m`` from the measured curvature vs.
         steer data using the bicycle model: curvature = tan(steer * gamma_max) / L.

    The fitted arm length is the calibration output. It replaces the default
    guess in the EnKF's BicycleModelConfig and the controller's steering
    gains for the subsequent mission.
    """

    test_throttle: float = 0.25
    straight_duration_s: float = 3.0
    steer_test_values: tuple[float, ...] = (0.3, 0.6, 1.0)
    steer_test_duration_s: float = 3.0
    assumed_max_gamma_rad: float = math.radians(45.0)


@dataclass
class CalibrationResult:
    """Output of the calibration routine."""

    effective_arm_length_m: float = 2.5
    steer_values: list[float] = field(default_factory=list)
    measured_curvatures: list[float] = field(default_factory=list)
    measured_speeds: list[float] = field(default_factory=list)
    fit_residual: float = 0.0


class CalibrationDriver:
    """
    Pre-mission calibration driver. Executes a short scripted maneuver to
    characterize the rover's actual steering response under current conditions
    (sled, snow, temperature), then fits an effective arm length.

    Usage::

        cal = CalibrationDriver(CalibrationConfig())
        while not cal.complete:
            cmd = cal.update(state, mission=None, dt=dt)
            # feed cmd to vehicle...
        result = cal.result
        # Use result.effective_arm_length_m for subsequent mission

    The calibration takes about ``straight_duration_s + len(steer_test_values)
    * steer_test_duration_s`` seconds of driving time.
    """

    def __init__(
        self,
        config: Optional[CalibrationConfig] = None,
        enkf: Optional[EnsembleKalmanFilter] = None,
    ) -> None:
        self.config = config or CalibrationConfig()
        self.enkf = enkf or EnsembleKalmanFilter()
        self._phase = CalibrationPhase.STRAIGHT
        self._phase_elapsed: float = 0.0
        self._test_idx: int = 0
        self._initialized: bool = False
        self._last_cmd: Optional[CommandBus] = None
        self._last_obs_t: float = -math.inf
        self._result: Optional[CalibrationResult] = None

        self._yaw_rate_samples: list[float] = []
        self._speed_samples: list[float] = []
        self._steer_curvatures: list[tuple[float, float, float]] = []

    # ---------- public API ----------

    @property
    def complete(self) -> bool:
        return self._phase is CalibrationPhase.COMPLETE

    @property
    def phase(self) -> CalibrationPhase:
        return self._phase

    @property
    def result(self) -> Optional[CalibrationResult]:
        return self._result

    def reset(self) -> None:
        self._phase = CalibrationPhase.STRAIGHT
        self._phase_elapsed = 0.0
        self._test_idx = 0
        self._initialized = False
        self._last_cmd = None
        self._last_obs_t = -math.inf
        self._result = None
        self._yaw_rate_samples.clear()
        self._speed_samples.clear()
        self._steer_curvatures.clear()
        self.enkf.reset()

    def update(
        self,
        state: RoverState,
        mission: Optional[Mission],
        dt: float,
    ) -> CommandBus:
        self._step_filter(state, dt)
        cmd = self._compute_command(dt)
        self._last_cmd = cmd
        return cmd

    # ---------- filter ----------

    def _step_filter(self, state: RoverState, dt: float) -> None:
        if not self._initialized:
            if state.fix_type is FixType.NONE:
                return
            x0 = np.array(
                [state.x, state.y, state.heading, state.speed, state.yaw_rate],
                dtype=float,
            )
            self.enkf.initialize(x0)
            self._initialized = True
            self._last_obs_t = state.t
            return

        self.enkf.predict(self._last_cmd, dt)
        if state.fix_type is not FixType.NONE and state.t > self._last_obs_t + 1e-9:
            self.enkf.update_from_gnss(state)
            self._last_obs_t = state.t

    # ---------- calibration state machine ----------

    def _compute_command(self, dt: float) -> CommandBus:
        if not self._initialized:
            return CommandBus(brake=0.5, direction_mode=DirectionMode.CLOSED_LOOP)

        if self._phase is CalibrationPhase.COMPLETE:
            return CommandBus(brake=0.5, direction_mode=DirectionMode.CLOSED_LOOP)

        self._phase_elapsed += dt
        cfg = self.config

        if self._phase is CalibrationPhase.STRAIGHT:
            if self._phase_elapsed >= cfg.straight_duration_s:
                self._phase = CalibrationPhase.STEER_TEST
                self._phase_elapsed = 0.0
                self._yaw_rate_samples.clear()
                self._speed_samples.clear()
            return CommandBus(
                throttle=cfg.test_throttle,
                steer=0.0,
                direction_mode=DirectionMode.CLOSED_LOOP,
            )

        if self._phase is CalibrationPhase.STEER_TEST:
            steer_val = cfg.steer_test_values[self._test_idx]

            fused = self.enkf.mean
            speed = float(fused[STATE_SPEED])
            yaw_rate = float(fused[STATE_YAW_RATE])

            if self._phase_elapsed > cfg.steer_test_duration_s * 0.3:
                self._yaw_rate_samples.append(yaw_rate)
                self._speed_samples.append(speed)

            if self._phase_elapsed >= cfg.steer_test_duration_s:
                if self._speed_samples:
                    avg_speed = np.mean(self._speed_samples)
                    avg_yaw = np.mean(self._yaw_rate_samples)
                    curvature = avg_yaw / max(abs(avg_speed), 0.01)
                    self._steer_curvatures.append(
                        (steer_val, float(curvature), float(avg_speed))
                    )

                self._test_idx += 1
                self._phase_elapsed = 0.0
                self._yaw_rate_samples.clear()
                self._speed_samples.clear()

                if self._test_idx >= len(cfg.steer_test_values):
                    self._phase = CalibrationPhase.FITTING
                    self._fit_model()
                    self._phase = CalibrationPhase.COMPLETE
                    return CommandBus(brake=0.5, direction_mode=DirectionMode.CLOSED_LOOP)

            return CommandBus(
                throttle=cfg.test_throttle,
                steer=steer_val,
                direction_mode=DirectionMode.CLOSED_LOOP,
            )

        return CommandBus(brake=0.5, direction_mode=DirectionMode.CLOSED_LOOP)

    # ---------- model fitting ----------

    def _fit_model(self) -> None:
        """
        Fit effective_arm_length from measured curvature vs. steer.

        Bicycle model: curvature = tan(steer * gamma_max) / L
        So:            L = tan(steer * gamma_max) / curvature

        We compute L for each test point and take the median (robust to outliers).
        """
        cfg = self.config
        L_estimates: list[float] = []
        steer_vals: list[float] = []
        curvatures: list[float] = []
        speeds: list[float] = []

        for steer, curv, spd in self._steer_curvatures:
            gamma = steer * cfg.assumed_max_gamma_rad
            tan_gamma = math.tan(gamma)
            if abs(curv) > 1e-6 and abs(tan_gamma) > 1e-6:
                L = tan_gamma / curv
                if L > 0:
                    L_estimates.append(L)
            steer_vals.append(steer)
            curvatures.append(curv)
            speeds.append(spd)

        if L_estimates:
            fitted_L = float(np.median(L_estimates))
            residuals = [abs(L - fitted_L) for L in L_estimates]
            fit_residual = float(np.mean(residuals))
        else:
            fitted_L = 2.5
            fit_residual = float("inf")

        self._result = CalibrationResult(
            effective_arm_length_m=fitted_L,
            steer_values=steer_vals,
            measured_curvatures=curvatures,
            measured_speeds=speeds,
            fit_residual=fit_residual,
        )
