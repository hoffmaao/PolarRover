"""Ensemble Kalman Filter for rover state estimation."""

from dataclasses import dataclass, field
from typing import Callable, Optional

import numpy as np

from rover_sim.control import CommandBus, FixType, RoverState

from rover_drive.estimation.state_model import (
    STATE_DIM,
    STATE_HEADING,
    STATE_SPEED,
    STATE_X,
    STATE_Y,
    STATE_YAW_RATE,
    BicycleStateModel,
)

Propagator = Callable[[np.ndarray, Optional[CommandBus], float], np.ndarray]


_DEFAULT_PROCESS_NOISE_STD: tuple[float, float, float, float, float] = (
    0.05,  # x   (m / sqrt(s))
    0.05,  # y   (m / sqrt(s))
    0.02,  # heading   (rad / sqrt(s))
    0.10,  # speed     (m/s / sqrt(s))
    0.05,  # yaw_rate  (rad/s / sqrt(s))
)


@dataclass
class EnKFConfig:
    """
    Ensemble Kalman filter configuration. Default values assume the state
    layout [x, y, heading, speed, yaw_rate] used by BicycleStateModel and
    the u-blox ZED-F9P observation envelopes from rover_sim's GNSS sensor.

    The process noise standard deviations are specified *per-sqrt-second*;
    they are scaled by sqrt(dt) at each predict() step, following the Wiener
    process convention.
    """

    ensemble_size: int = 64
    process_noise_std: tuple[float, float, float, float, float] = _DEFAULT_PROCESS_NOISE_STD
    initial_pos_std_m: float = 0.5
    initial_heading_std_rad: float = 0.1
    initial_speed_std_mps: float = 0.2
    initial_yaw_rate_std_rad_s: float = 0.1
    seed: Optional[int] = None


class EnsembleKalmanFilter:
    """
    Stochastic Ensemble Kalman Filter estimating rover state from GNSS (and
    later, IMU / inter-rover) observations.

    State vector: ``[x, y, heading, speed, yaw_rate]``.
    Observations (for now): 2D GNSS position ``[x, y]``.

    The forward model is supplied as a callable propagator; by default
    :class:`BicycleStateModel` is used. The stochastic EnKF variant is
    implemented here — each update step draws a fresh perturbed observation
    per ensemble member, which is the simplest and most robust formulation at
    our ensemble sizes (~50–200 members). A deterministic ETKF / EAKF variant
    can be added later without changing the public interface if small-ensemble
    bias becomes a concern.

    Typical usage inside a Driver::

        enkf = EnsembleKalmanFilter(EnKFConfig(seed=42))
        enkf.initialize(np.array([x0, y0, heading0, v0, w0]))
        for each time step:
            enkf.predict(cmd=last_cmd, dt=dt)
            enkf.update_from_gnss(rover_state)
            fused = enkf.mean
            # drive the controller off `fused` instead of raw rover_state
    """

    def __init__(
        self,
        config: Optional[EnKFConfig] = None,
        propagator: Optional[Propagator] = None,
    ) -> None:
        self.config = config or EnKFConfig()
        self.propagator: Propagator = propagator or BicycleStateModel().propagate
        self._rng = np.random.default_rng(self.config.seed)
        self._ensemble = np.zeros((self.config.ensemble_size, STATE_DIM))
        self._initialized = False

    # ---------- public API ----------

    @property
    def initialized(self) -> bool:
        return self._initialized

    @property
    def ensemble(self) -> np.ndarray:
        """The (N × state_dim) ensemble array. Returned as a view; don't mutate in-place."""
        return self._ensemble

    @property
    def mean(self) -> np.ndarray:
        return self._ensemble.mean(axis=0)

    @property
    def covariance(self) -> np.ndarray:
        X_prime = self._ensemble - self.mean
        return (X_prime.T @ X_prime) / (self.config.ensemble_size - 1)

    def reset(self) -> None:
        self._initialized = False
        self._ensemble = np.zeros((self.config.ensemble_size, STATE_DIM))
        self._rng = np.random.default_rng(self.config.seed)

    def initialize(self, x0: np.ndarray) -> None:
        """Sample the ensemble around a point estimate using the config stddevs."""
        cfg = self.config
        x0 = np.asarray(x0, dtype=float)
        if x0.shape != (STATE_DIM,):
            raise ValueError(f"x0 must have shape ({STATE_DIM},), got {x0.shape}")
        init_std = np.array(
            [
                cfg.initial_pos_std_m,
                cfg.initial_pos_std_m,
                cfg.initial_heading_std_rad,
                cfg.initial_speed_std_mps,
                cfg.initial_yaw_rate_std_rad_s,
            ]
        )
        perturbations = self._rng.standard_normal((cfg.ensemble_size, STATE_DIM)) * init_std
        self._ensemble = x0 + perturbations
        self._initialized = True

    def predict(self, cmd: Optional[CommandBus], dt: float) -> None:
        """Propagate every ensemble member forward by dt seconds with process noise."""
        if not self._initialized:
            raise RuntimeError("EnKF must be initialized before predict()")
        if dt <= 0.0:
            return
        cfg = self.config
        process_std = np.asarray(cfg.process_noise_std, dtype=float) * np.sqrt(dt)

        for i in range(cfg.ensemble_size):
            propagated = self.propagator(self._ensemble[i], cmd, dt)
            noise = self._rng.standard_normal(STATE_DIM) * process_std
            self._ensemble[i] = propagated + noise

    def update(self, observation: np.ndarray, obs_noise_std: float) -> None:
        """
        Fuse a 2D GNSS position observation (stochastic EnKF update).

        ``observation`` is the measurement vector ``[x, y]``. ``obs_noise_std``
        is the horizontal 1-sigma noise of the receiver; if it is <= 0, the
        observation is treated as noiseless and all ensemble members are
        collapsed onto it (useful for testing and for perfect-knowledge
        synthetic runs).
        """
        if not self._initialized:
            raise RuntimeError("EnKF must be initialized before update()")
        observation = np.asarray(observation, dtype=float)
        if observation.shape != (2,):
            raise ValueError(f"observation must have shape (2,), got {observation.shape}")

        if obs_noise_std <= 0.0:
            self._ensemble[:, STATE_X] = observation[0]
            self._ensemble[:, STATE_Y] = observation[1]
            return

        cfg = self.config
        N = cfg.ensemble_size

        # Observation operator is H = [[1,0,0,0,0],[0,1,0,0,0]]; that is, the
        # observation space equals the first two state components. So we don't
        # need to form H explicitly — we slice the ensemble instead.
        mean_state = self.mean
        X_prime = self._ensemble - mean_state              # (N, STATE_DIM)
        HX = self._ensemble[:, :2]                          # (N, 2)
        HX_prime = HX - mean_state[:2]                      # (N, 2)

        P_xy = (X_prime.T @ HX_prime) / (N - 1)             # (STATE_DIM, 2)
        P_yy = (HX_prime.T @ HX_prime) / (N - 1)            # (2, 2)

        R = np.eye(2) * (obs_noise_std**2)
        S = P_yy + R                                        # (2, 2)
        K = P_xy @ np.linalg.inv(S)                         # (STATE_DIM, 2)

        # Stochastic EnKF: perturb the observation per ensemble member
        obs_noise = self._rng.standard_normal((N, 2)) * obs_noise_std
        perturbed_obs = observation + obs_noise             # (N, 2)
        innovations = perturbed_obs - HX                    # (N, 2)
        self._ensemble = self._ensemble + innovations @ K.T  # (N, STATE_DIM)

    def update_from_gnss(self, state: RoverState) -> bool:
        """
        Convenience wrapper: fuse a :class:`RoverState` produced by the GNSS
        sensor. Returns True if the update was applied, or False if the
        observation was skipped because the fix was NONE (no valid position).
        """
        if state.fix_type is FixType.NONE:
            return False
        self.update(
            observation=np.array([state.x, state.y]),
            obs_noise_std=state.position_std_m,
        )
        return True

    # ---------- diagnostic helpers ----------

    def position_std(self) -> tuple[float, float]:
        """Return the current (sigma_x, sigma_y) from the ensemble covariance."""
        P = self.covariance
        return float(np.sqrt(P[STATE_X, STATE_X])), float(np.sqrt(P[STATE_Y, STATE_Y]))
