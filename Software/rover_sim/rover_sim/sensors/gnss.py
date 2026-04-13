import math
import random
from dataclasses import dataclass, field
from typing import Optional

from rover_sim.control.state import FixType, RoverState
from rover_sim.vehicle.base import VehicleState


# Default horizontal position sigma for each fix type, tuned to the u-blox
# ZED-F9P's typical performance envelopes.
FIX_NOISE_STD_M: dict[FixType, float] = {
    FixType.FIXED: 0.015,  # RTK fixed: 1-2 cm
    FixType.FLOAT: 0.15,   # RTK float: 10-30 cm
    FixType.DGPS: 0.75,    # DGPS / SBAS: 0.5-1 m
    FixType.SPP: 1.5,      # standalone: 1-2 m
    FixType.NONE: 0.0,     # no fix reported
}


@dataclass
class GnssConfig:
    """
    Parameterized GNSS sensor model tuned for the u-blox ZED-F9P dual-band
    RTK receiver. By default the sensor emits FIXED-quality measurements at
    10 Hz. The per-step transition probabilities (downgrade / upgrade / drop)
    let tests reproduce realistic degradation so drive code is forced to
    tolerate float / dgps / spp / dropout states.
    """

    rate_hz: float = 10.0
    default_fix: FixType = FixType.FIXED
    noise_std_m: Optional[dict[FixType, float]] = None
    prob_downgrade: float = 0.0
    prob_upgrade: float = 0.0
    prob_dropout: float = 0.0
    seed: Optional[int] = None


class GnssSensor:
    """
    Consumes ground-truth VehicleState and emits noisy, rate-limited
    RoverState observations, modeling the ZED-F9P fix-type ladder.

    Velocity and yaw-rate fields are passed through from truth for now; those
    come from an IMU on the real vehicle and will get their own sensor model
    once we add one. The sim's goal for Phase 2 is to make *position* noise
    and fix-type behavior the dominant realism, since that's what the drive
    stack has to tolerate.
    """

    def __init__(self, config: Optional[GnssConfig] = None) -> None:
        self.config = config or GnssConfig()
        self._rng = random.Random(self.config.seed)
        self._current_fix: FixType = self.config.default_fix
        self._last_emit_t: float = -math.inf

    def reset(self) -> None:
        self._rng = random.Random(self.config.seed)
        self._current_fix = self.config.default_fix
        self._last_emit_t = -math.inf

    def sample(self, truth: VehicleState, t: float) -> Optional[RoverState]:
        """
        Return a noisy RoverState for time `t`, or None if the receiver is
        rate-limited and would not produce a sample this dt.
        """
        period = 1.0 / self.config.rate_hz
        if t - self._last_emit_t < period - 1e-9:
            return None
        self._last_emit_t = t

        self._evolve_fix()
        std = self._noise_std(self._current_fix)

        if self._current_fix is FixType.NONE:
            return RoverState(
                t=t,
                x=truth.x,
                y=truth.y,
                heading=truth.heading,
                speed=truth.speed,
                yaw_rate=truth.yaw_rate,
                fix_type=FixType.NONE,
                position_std_m=0.0,
            )

        nx = truth.x + self._rng.gauss(0.0, std)
        ny = truth.y + self._rng.gauss(0.0, std)
        return RoverState(
            t=t,
            x=nx,
            y=ny,
            heading=truth.heading,
            speed=truth.speed,
            yaw_rate=truth.yaw_rate,
            fix_type=self._current_fix,
            position_std_m=std,
        )

    @property
    def current_fix(self) -> FixType:
        return self._current_fix

    def _noise_std(self, fix: FixType) -> float:
        overrides = self.config.noise_std_m
        if overrides is not None and fix in overrides:
            return overrides[fix]
        return FIX_NOISE_STD_M[fix]

    def _evolve_fix(self) -> None:
        cfg = self.config
        if cfg.prob_downgrade == cfg.prob_upgrade == cfg.prob_dropout == 0.0:
            return  # deterministic-stable mode

        r = self._rng.random()
        if r < cfg.prob_dropout:
            self._current_fix = FixType.NONE
            return

        if self._current_fix is FixType.NONE:
            # Try to re-acquire as SPP; further upgrades happen on subsequent samples
            if r < cfg.prob_dropout + cfg.prob_upgrade:
                self._current_fix = FixType.SPP
            return

        thresh_down = cfg.prob_dropout + cfg.prob_downgrade
        thresh_up = thresh_down + cfg.prob_upgrade
        if r < thresh_down:
            self._current_fix = _worse_fix(self._current_fix)
        elif r < thresh_up:
            self._current_fix = _better_fix(self._current_fix)


_FIX_ORDER: list[FixType] = [
    FixType.NONE,
    FixType.SPP,
    FixType.DGPS,
    FixType.FLOAT,
    FixType.FIXED,
]


def _worse_fix(fix: FixType) -> FixType:
    i = _FIX_ORDER.index(fix)
    return _FIX_ORDER[max(0, i - 1)]


def _better_fix(fix: FixType) -> FixType:
    i = _FIX_ORDER.index(fix)
    return _FIX_ORDER[min(len(_FIX_ORDER) - 1, i + 1)]
