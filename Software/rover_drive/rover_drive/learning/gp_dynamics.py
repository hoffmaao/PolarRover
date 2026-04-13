"""GP-based dynamics model for rover steering — learns the mapping from
steer command to resulting curvature (yaw_rate / speed) from driving data.

This replaces the fixed bicycle-model assumption (curvature = tan(steer * gamma_max) / L)
with a learned, nonparametric model that adapts to the actual vehicle + sled + snow
conditions. The GP provides:
  - A mean prediction (best estimate of curvature for a given steer)
  - An uncertainty estimate (how confident the model is at this steer value)

The MPC can use both: steer through regions of high confidence, and be
cautious (slower, more conservative) in regions of high uncertainty.
"""

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from rover_drive.learning.gp import GaussianProcess, RBFKernel


@dataclass
class GPDynamicsConfig:
    """Configuration for the GP dynamics learner."""

    kernel_length_scale: float = 0.3
    kernel_variance: float = 0.5
    noise_var: float = 0.005
    min_speed_for_learning: float = 0.2
    max_training_points: int = 500
    refit_interval: int = 10


class GPDynamicsModel:
    """
    Learns the rover's steer → curvature mapping from driving data.

    Call ``observe(steer, speed, yaw_rate)`` after each simulation step to
    feed data into the model. Call ``predict_curvature(steer)`` to get the
    GP's mean and variance prediction for a steer command.

    The model is initialized empty (high uncertainty everywhere). After a
    rosette calibration phase that exercises a range of steer values, the
    GP tightens up and the MPC starts producing better predictions.
    """

    def __init__(self, config: Optional[GPDynamicsConfig] = None) -> None:
        self.config = config or GPDynamicsConfig()
        self.gp = GaussianProcess(
            kernel=RBFKernel(
                length_scale=self.config.kernel_length_scale,
                variance=self.config.kernel_variance,
            ),
            noise_var=self.config.noise_var,
        )
        self._buffer_x: list[float] = []
        self._buffer_y: list[float] = []
        self._obs_count: int = 0
        self._needs_refit: bool = False

    @property
    def n_observations(self) -> int:
        return self.gp.n_samples + len(self._buffer_x)

    @property
    def fitted(self) -> bool:
        return self.gp.fitted

    def observe(self, steer: float, speed: float, yaw_rate: float) -> None:
        """Record one (steer, observed_curvature) data point from driving."""
        if abs(speed) < self.config.min_speed_for_learning:
            return
        curvature = yaw_rate / speed
        self._buffer_x.append(steer)
        self._buffer_y.append(curvature)
        self._obs_count += 1

        if self._obs_count % self.config.refit_interval == 0:
            self._flush_buffer()

    def predict_curvature(self, steer: float) -> tuple[float, float]:
        """Predict (mean_curvature, variance) for a given steer command.
        Uses the last fitted GP without flushing pending observations —
        the refit_interval schedule handles batch updates."""
        return self.gp.predict_single(np.array([[steer]]))

    def predict_curvature_batch(self, steers: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Batch prediction for multiple steer values."""
        return self.gp.predict(steers.reshape(-1, 1))

    def forward_predict(
        self,
        x: float, y: float, heading: float, speed: float,
        steer: float, dt: float,
    ) -> tuple[float, float, float, float]:
        """
        Predict the next state using the GP-learned curvature model.
        Returns (x_new, y_new, heading_new, model_uncertainty).
        """
        mean_curv, var_curv = self.predict_curvature(steer)
        yaw_rate = mean_curv * speed

        h_new = heading + yaw_rate * dt
        h_avg = 0.5 * (heading + h_new)
        x_new = x + speed * math.cos(h_avg) * dt
        y_new = y + speed * math.sin(h_avg) * dt

        uncertainty = math.sqrt(max(var_curv, 0)) * abs(speed) * dt
        return x_new, y_new, h_new, uncertainty

    def forward_predict_sequence(
        self,
        x: float, y: float, heading: float, speed: float,
        steers: list[float], dt: float,
    ) -> list[tuple[float, float, float, float]]:
        """Batch forward prediction for a steer sequence — precomputes GP
        predictions for all unique steers to avoid repeated kernel evaluations."""
        unique_steers = list(set(steers))
        steer_arr = np.array(unique_steers).reshape(-1, 1)
        means, vars_ = self.predict_curvature_batch(steer_arr)
        lookup = {s: (float(means[i]), float(vars_[i])) for i, s in enumerate(unique_steers)}

        result = []
        for s in steers:
            mc, vc = lookup[s]
            yr = mc * speed
            h_new = heading + yr * dt
            h_avg = 0.5 * (heading + h_new)
            x = x + speed * math.cos(h_avg) * dt
            y = y + speed * math.sin(h_avg) * dt
            heading = h_new
            unc = math.sqrt(max(vc, 0)) * abs(speed) * dt
            result.append((x, y, heading, unc))
        return result

    def reset(self) -> None:
        self.gp.reset()
        self._buffer_x.clear()
        self._buffer_y.clear()
        self._obs_count = 0

    def _flush_buffer(self) -> None:
        if not self._buffer_x:
            return
        cfg = self.config
        X = np.array(self._buffer_x).reshape(-1, 1)
        y = np.array(self._buffer_y)
        self.gp.add_data(X, y)
        self._buffer_x.clear()
        self._buffer_y.clear()

        # Trim to max training points (keep most recent)
        if self.gp.n_samples > cfg.max_training_points:
            keep = cfg.max_training_points
            self.gp.fit(self.gp.X_train[-keep:], self.gp.y_train[-keep:])

    def _flush_buffer_if_needed(self) -> None:
        if self._buffer_x:
            self._flush_buffer()
