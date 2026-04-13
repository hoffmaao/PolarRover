"""Scenario runner — the main event loop that ties sim + driver together."""

import math
from pathlib import Path
from typing import Any, Optional

from rover_sim.config import ScenarioConfig
from rover_sim.control import CommandBus, Direction, DirectionMode, Driver, FixType, RoverState
from rover_sim.missions import Mission, load_mission
from rover_sim.safety import SafetyConfig, SafetyFilter
from rover_sim.sensors import GnssConfig, GnssSensor
from rover_sim.vehicle import (
    SideBySideSkidSteer,
    SideBySideSkidSteerConfig,
    SingleTrackArticulated,
    SingleTrackArticulatedConfig,
)
from rover_sim.vehicle.base import Vehicle, VehicleState

from rover_sim_emulator.drivers import create_driver
from rover_sim_emulator.logger import TelemetryLogger


_FIX_NAME_TO_TYPE = {
    "fixed": FixType.FIXED,
    "float": FixType.FLOAT,
    "dgps": FixType.DGPS,
    "spp": FixType.SPP,
    "none": FixType.NONE,
}


class ScenarioRunner:
    """
    Loads a ScenarioConfig, instantiates a rover_sim vehicle + safety filter +
    GNSS sensor + rover_drive Driver, runs the simulation to completion, and
    writes a JSONL telemetry log. Returns a summary dict from ``run()``.

    This is the thing ``rover-sim-emu run scenario.yaml`` invokes. Every
    downstream phase (waypoint, multipass, linked-CMP) plugs in via the
    driver factory — the runner itself stays mission-kind agnostic.
    """

    def __init__(self, config: ScenarioConfig) -> None:
        self.config = config
        self.mission: Optional[Mission] = (
            load_mission(config.mission_path) if config.mission_path else None
        )
        self.vehicle: Vehicle = self._build_vehicle()
        self.safety = SafetyFilter(self._build_safety_config())
        self.gnss = GnssSensor(self._build_gnss_config())
        self.driver: Driver = create_driver(
            kind=config.controller.kind,
            params=config.controller.params,
            mission=self.mission,
        )
        self.logger = TelemetryLogger(self._resolve_log_path())
        self._t: float = 0.0
        self._last_rover_state: Optional[RoverState] = None

    # ---------- public loop ----------

    def run(self) -> dict[str, Any]:
        n_steps = int(round(self.config.duration_s / self.config.dt_s))
        with self.logger:
            for _ in range(n_steps):
                self.step(self.config.dt_s)
        return self._summary()

    def step(self, dt: float) -> None:
        vstate = self.vehicle.state
        sample = self.gnss.sample(vstate, self._t)
        if sample is not None:
            self._last_rover_state = sample
        rover_state = self._last_rover_state or RoverState(
            t=self._t, fix_type=FixType.NONE
        )

        cmd = self.driver.update(rover_state, self.mission, dt)
        cmd = self.safety.filter(cmd, vstate, dt)
        self.vehicle.step(cmd, dt)

        fused = self._extract_fused()
        self.logger.log(t=self._t, truth=vstate, rover_state=rover_state, cmd=cmd, fused=fused)
        self._t += dt

    def _extract_fused(self) -> Optional[dict[str, float]]:
        """Try to read the EnKF fused estimate from the driver, if it has one."""
        enkf = getattr(self.driver, "enkf", None)
        if enkf is None or not getattr(enkf, "initialized", False):
            return None
        from rover_drive.estimation import STATE_X, STATE_Y, STATE_HEADING, STATE_SPEED, STATE_YAW_RATE
        m = enkf.mean
        sx, sy = enkf.position_std()
        return {
            "x": float(m[STATE_X]),
            "y": float(m[STATE_Y]),
            "heading": float(m[STATE_HEADING]),
            "speed": float(m[STATE_SPEED]),
            "yaw_rate": float(m[STATE_YAW_RATE]),
            "pos_std_x": sx,
            "pos_std_y": sy,
        }

    # ---------- builders ----------

    def _build_vehicle(self) -> Vehicle:
        vcfg = self.config.vehicle
        kc = vcfg.kinematics
        init_state = VehicleState(
            x=vcfg.initial_pose.x,
            y=vcfg.initial_pose.y,
            heading=vcfg.initial_pose.heading_rad,
        )

        if vcfg.type == "single_track":
            vehicle: Vehicle = SingleTrackArticulated(
                SingleTrackArticulatedConfig(
                    arm_length_m=kc.arm_length_m,
                    max_speed_mps=kc.max_speed_mps,
                    max_gamma_rad=math.radians(kc.max_gamma_deg),
                    accel_mps2=kc.accel_mps2,
                    brake_decel_mps2=kc.brake_decel_mps2,
                )
            )
        elif vcfg.type in ("side_by_side_left", "side_by_side_right"):
            vehicle = SideBySideSkidSteer(
                SideBySideSkidSteerConfig(
                    track_width_m=kc.track_width_m,
                    max_speed_mps=kc.max_speed_mps,
                    accel_mps2=kc.accel_mps2,
                    brake_decel_mps2=kc.brake_decel_mps2,
                )
            )
        else:
            raise ValueError(
                f"unknown vehicle type {vcfg.type!r}. "
                f"Available: single_track, side_by_side_left, side_by_side_right"
            )

        vehicle.reset(init_state)
        return vehicle

    def _build_safety_config(self) -> SafetyConfig:
        s = self.config.vehicle.safety
        return SafetyConfig(
            max_speed_mps=s.max_speed_mps,
            learning_max_speed_mps=s.learning_max_speed_mps,
            learning_mode=s.learning_mode,
            auto_neutral_timeout_s=s.auto_neutral_timeout_s,
            f_r_stop_eps_mps=s.f_r_stop_eps_mps,
        )

    def _build_gnss_config(self) -> GnssConfig:
        g = self.config.vehicle.gnss
        default_fix = _FIX_NAME_TO_TYPE.get(g.default_fix.lower())
        if default_fix is None:
            raise ValueError(
                f"unknown gnss default_fix {g.default_fix!r}. "
                f"Available: {list(_FIX_NAME_TO_TYPE)}"
            )
        return GnssConfig(
            rate_hz=g.rate_hz,
            default_fix=default_fix,
            prob_dropout=g.prob_dropout,
            prob_downgrade=g.prob_downgrade,
            prob_upgrade=g.prob_upgrade,
            seed=self.config.seed,
        )

    def _resolve_log_path(self) -> Path:
        if self.config.log_path:
            return Path(self.config.log_path)
        return Path("./runs") / f"{self.config.name}.jsonl"

    # ---------- summary ----------

    def _summary(self) -> dict[str, Any]:
        s = self.vehicle.state
        return {
            "name": self.config.name,
            "controller": self.config.controller.kind,
            "duration_s": self._t,
            "log_path": str(self.logger.path),
            "final_x": s.x,
            "final_y": s.y,
            "final_heading_rad": s.heading,
            "final_speed_mps": s.speed,
        }
