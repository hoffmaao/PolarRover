"""Minimal Gaussian Process regression — numpy only, no scikit-learn dependency.

A GP is a non-parametric probabilistic model: given training points
{(x_i, y_i)}, it predicts the output distribution at new inputs x* as:

    mean(x*)  = K(x*, X) @ (K(X, X) + σ²I)⁻¹ @ y
    var(x*)   = K(x*, x*) - K(x*, X) @ (K(X, X) + σ²I)⁻¹ @ K(X, x*)

Key properties for rover drive:
  - Learns from data with no assumed functional form
  - Provides calibrated uncertainty (MPC knows where it's guessing)
  - Improves online as new driving data arrives
  - Works well in low dimensions (our inputs are 1–3D)
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional


@dataclass
class RBFKernel:
    """Squared exponential (RBF / Gaussian) kernel: k(x, x') = σ² exp(-||x-x'||² / 2l²)"""

    length_scale: float = 1.0
    variance: float = 1.0

    def __call__(self, X1: np.ndarray, X2: np.ndarray) -> np.ndarray:
        X1 = np.atleast_2d(X1)
        X2 = np.atleast_2d(X2)
        d2 = np.sum(X1**2, axis=1, keepdims=True) + np.sum(X2**2, axis=1) - 2 * X1 @ X2.T
        return self.variance * np.exp(-0.5 * d2 / (self.length_scale**2))


class GaussianProcess:
    """
    Gaussian Process regressor. Fits on (X, y) data and predicts mean + variance
    at new test points. Supports online data addition via ``add_data()``.

    For the rover dynamics learning use case, this typically maps:
      - Input: [steer_command] or [steer_command, speed] (1–2D)
      - Output: measured curvature (yaw_rate / speed) (scalar)
    """

    def __init__(
        self,
        kernel: Optional[RBFKernel] = None,
        noise_var: float = 0.01,
    ) -> None:
        self.kernel = kernel or RBFKernel()
        self.noise_var = noise_var
        self.X_train: Optional[np.ndarray] = None
        self.y_train: Optional[np.ndarray] = None
        self._alpha: Optional[np.ndarray] = None  # K_inv @ y for fast prediction
        self._K_inv: Optional[np.ndarray] = None

    @property
    def n_samples(self) -> int:
        return 0 if self.X_train is None else len(self.X_train)

    @property
    def fitted(self) -> bool:
        return self._alpha is not None

    def fit(self, X: np.ndarray, y: np.ndarray) -> None:
        """Fit the GP on training data X (n × d) and y (n,)."""
        X = np.atleast_2d(X)
        y = np.asarray(y, dtype=float).ravel()
        self.X_train = X
        self.y_train = y
        K = self.kernel(X, X) + self.noise_var * np.eye(len(X))
        self._K_inv = np.linalg.inv(K)
        self._alpha = self._K_inv @ y

    def predict(self, X_test: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Predict mean and variance at test points. Returns (mean, var)."""
        X_test = np.atleast_2d(X_test)
        if not self.fitted:
            mean = np.zeros(len(X_test))
            var = np.full(len(X_test), self.kernel.variance)
            return mean, var

        K_star = self.kernel(X_test, self.X_train)
        mean = K_star @ self._alpha
        K_ss = self.kernel(X_test, X_test)
        var = np.diag(K_ss - K_star @ self._K_inv @ K_star.T)
        var = np.maximum(var, 0.0)  # numerical stability
        return mean, var

    def predict_single(self, x: np.ndarray) -> tuple[float, float]:
        """Convenience: predict at a single point, returns (mean, var) as floats."""
        m, v = self.predict(np.atleast_2d(x))
        return float(m[0]), float(v[0])

    def add_data(self, X_new: np.ndarray, y_new: np.ndarray) -> None:
        """Add new observations and refit. For online learning."""
        X_new = np.atleast_2d(X_new)
        y_new = np.asarray(y_new, dtype=float).ravel()
        if self.X_train is None:
            self.fit(X_new, y_new)
        else:
            X = np.vstack([self.X_train, X_new])
            y = np.concatenate([self.y_train, y_new])
            self.fit(X, y)

    def reset(self) -> None:
        self.X_train = None
        self.y_train = None
        self._alpha = None
        self._K_inv = None
