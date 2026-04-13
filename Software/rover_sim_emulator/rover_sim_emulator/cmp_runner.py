"""Dual-vehicle scenario runner for linked CMP surveys."""

import math
from pathlib import Path
from typing import Any, Optional

from rover_sim.config import ScenarioConfig
from rover_sim.control import FixType, RoverState
from rover_sim.missions import Mission, Waypoint, load_mission
from rover_sim.safety import SafetyConfig, SafetyFilter
from rover_sim.sensors import GnssConfig, GnssSensor
from rover_sim.vehicle import (
    SingleTrackArticulated,
    SingleTrackArticulatedConfig,
)
from rover_sim.vehicle.base import Vehicle, VehicleState

from rover_drive.estimation import EnKFConfig, EnsembleKalmanFilter
from rover_drive.modes.linked_cmp import CMPFormationConfig, LinkedCMPDriver

from rover_sim_emulator.logger import TelemetryLogger, _to_jsonable


_FIX_NAME_TO_TYPE = {
    "fixed": FixType.FIXED,
    "float": FixType.FLOAT,
    "dgps": FixType.DGPS,
    "spp": FixType.SPP,
    "none": FixType.NONE,
}


class LinkedCMPRunner:
    """
    Dual-vehicle scenario runner for linked CMP surveys. Manages two
    identical rovers (A and B) whose initial poses are computed from the
    mission centerline + the configured spread distance. Runs a single
    :class:`LinkedCMPDriver` that produces per-rover commands each step.

    The single ``vehicle`` block in the scenario config is used as the
    template for both rovers — they share kinematics, safety, and GNSS
    settings. The only per-rover difference is the initial pose (one on each
    side of the centerline, perpendicular at ``spread_distance_m / 2``).
    """

    def __init__(self, config: ScenarioConfig) -> None:
        self.config = config
        if not config.mission_path:
            raise ValueError("linked_cmp requires a mission_path in the scenario config")
        self.mission: Mission = load_mission(config.mission_path)
        if len(self.mission.waypoints) < 2:
            raise ValueError("linked_cmp mission must have at least 2 centerline points")

        params = config.controller.params
        spread = float(params.get("start_spread_m", 2.0))

        pose_a, pose_b = self._compute_initial_poses(spread)
        self.vehicle_a: Vehicle = self._build_vehicle(pose_a)
        self.vehicle_b: Vehicle = self._build_vehicle(pose_b)
        self.gnss_a = GnssSensor(self._build_gnss_config())
        self.gnss_b = GnssSensor(self._build_gnss_config())
        self.safety_a = SafetyFilter(self._build_safety_config())
        self.safety_b = SafetyFilter(self._build_safety_config())

        self.driver: LinkedCMPDriver = self._build_driver(params)
        self.logger = TelemetryLogger(self._resolve_log_path())

        self._t: float = 0.0
        self._last_state_a: Optional[RoverState] = None
        self._last_state_b: Optional[RoverState] = None

    # ---------- public ----------

    def run(self) -> dict[str, Any]:
        n_steps = int(round(self.config.duration_s / self.config.dt_s))
        with self.logger:
            for _ in range(n_steps):
                self._step(self.config.dt_s)
        return self._summary()

    # ---------- step ----------

    def _step(self, dt: float) -> None:
        vs_a = self.vehicle_a.state
        vs_b = self.vehicle_b.state

        sample_a = self.gnss_a.sample(vs_a, self._t)
        sample_b = self.gnss_b.sample(vs_b, self._t)
        if sample_a is not None:
            self._last_state_a = sample_a
        if sample_b is not None:
            self._last_state_b = sample_b

        rs_a = self._last_state_a or RoverState(t=self._t, fix_type=FixType.NONE)
        rs_b = self._last_state_b or RoverState(t=self._t, fix_type=FixType.NONE)

        cmd_a, cmd_b = self.driver.update(rs_a, rs_b, self.mission, dt)
        cmd_a = self.safety_a.filter(cmd_a, vs_a, dt)
        cmd_b = self.safety_b.filter(cmd_b, vs_b, dt)
        self.vehicle_a.step(cmd_a, dt)
        self.vehicle_b.step(cmd_b, dt)

        self.logger.log_record({
            "t": self._t,
            "rover_a": {
                "truth": _to_jsonable(vs_a),
                "rover_state": _to_jsonable(rs_a),
                "cmd": _to_jsonable(cmd_a),
            },
            "rover_b": {
                "truth": _to_jsonable(vs_b),
                "rover_state": _to_jsonable(rs_b),
                "cmd": _to_jsonable(cmd_b),
            },
            "formation": {
                "midpoint_x": (vs_a.x + vs_b.x) / 2,
                "midpoint_y": (vs_a.y + vs_b.y) / 2,
                "actual_spread_m": math.hypot(vs_a.x - vs_b.x, vs_a.y - vs_b.y),
            },
        })
        self._t += dt

    # ---------- builders ----------

    def _compute_initial_poses(
        self, spread: float
    ) -> tuple[VehicleState, VehicleState]:
        """Place rovers along the survey axis at ± start_spread/2 from the
        midpoint (wp[0]), heading outward in opposite directions."""
        wps = self.mission.waypoints
        p0 = wps[0]  # the fixed midpoint M
        p1 = wps[1]  # defines the survey direction
        dx = p1.x - p0.x
        dy = p1.y - p0.y
        length = max(1e-12, math.hypot(dx, dy))
        sx, sy = dx / length, dy / length  # survey direction unit vector
        heading_a = math.atan2(sy, sx)       # rover A heads outward in +dir
        heading_b = heading_a + math.pi      # rover B heads outward in -dir
        half = spread / 2.0

        pose_a = VehicleState(
            x=p0.x + half * sx,
            y=p0.y + half * sy,
            heading=heading_a,
        )
        pose_b = VehicleState(
            x=p0.x - half * sx,
            y=p0.y - half * sy,
            heading=heading_b,
        )
        return pose_a, pose_b

    def _build_vehicle(self, initial: VehicleState) -> Vehicle:
        kc = self.config.vehicle.kinematics
        v = SingleTrackArticulated(
            SingleTrackArticulatedConfig(
                arm_length_m=kc.arm_length_m,
                max_speed_mps=kc.max_speed_mps,
                max_gamma_rad=math.radians(kc.max_gamma_deg),
                accel_mps2=kc.accel_mps2,
                brake_decel_mps2=kc.brake_decel_mps2,
            )
        )
        v.reset(initial)
        return v

    def _build_gnss_config(self) -> GnssConfig:
        g = self.config.vehicle.gnss
        return GnssConfig(
            rate_hz=g.rate_hz,
            default_fix=_FIX_NAME_TO_TYPE.get(g.default_fix.lower(), FixType.FIXED),
            prob_dropout=g.prob_dropout,
            prob_downgrade=g.prob_downgrade,
            prob_upgrade=g.prob_upgrade,
            seed=self.config.seed,
        )

    def _build_safety_config(self) -> SafetyConfig:
        s = self.config.vehicle.safety
        return SafetyConfig(
            max_speed_mps=s.max_speed_mps,
            learning_max_speed_mps=s.learning_max_speed_mps,
            learning_mode=s.learning_mode,
            auto_neutral_timeout_s=s.auto_neutral_timeout_s,
            f_r_stop_eps_mps=s.f_r_stop_eps_mps,
        )

    def _build_driver(self, params: dict[str, Any]) -> LinkedCMPDriver:
        formation_config = CMPFormationConfig()
        for key in (
            "start_spread_m",
            "end_spread_m",
            "spread_rate_m_per_s",
            "cruise_throttle",
            "approach_throttle",
            "target_tolerance_m",
        ):
            if key in params:
                setattr(formation_config, key, float(params[key]))
        if "heading_saturation_deg" in params:
            formation_config.heading_saturation_rad = math.radians(
                float(params["heading_saturation_deg"])
            )

        seed = params.get("enkf_seed")
        enkf_a = EnsembleKalmanFilter(EnKFConfig(seed=seed)) if seed is not None else None
        enkf_b = EnsembleKalmanFilter(
            EnKFConfig(seed=seed + 1 if seed is not None else None)
        )
        return LinkedCMPDriver(
            config=formation_config, enkf_a=enkf_a, enkf_b=enkf_b
        )

    def _resolve_log_path(self) -> Path:
        if self.config.log_path:
            return Path(self.config.log_path)
        return Path("./runs") / f"{self.config.name}_cmp.jsonl"

    # ---------- summary ----------

    def _summary(self) -> dict[str, Any]:
        sa = self.vehicle_a.state
        sb = self.vehicle_b.state
        m = self.driver.metrics
        return {
            "name": self.config.name,
            "controller": "linked_cmp",
            "duration_s": self._t,
            "log_path": str(self.logger.path),
            "rover_a_final": (sa.x, sa.y),
            "rover_b_final": (sb.x, sb.y),
            "midpoint_drift_rms_m": m.midpoint_drift_rms_m,
            "midpoint_drift_max_m": m.midpoint_drift_max_m,
            "spread_error_rms_m": m.spread_error_rms_m,
            "spread_error_max_m": m.spread_error_max_m,
            "final_spread_m": math.hypot(sa.x - sb.x, sa.y - sb.y),
            "target_spread_m": self.driver.current_target_spread,
            "complete": self.driver.complete,
        }
