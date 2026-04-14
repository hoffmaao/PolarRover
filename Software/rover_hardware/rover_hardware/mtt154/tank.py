"""Two MTT-154s coupled side-by-side as tank tracks, on two CAN buses."""

from dataclasses import dataclass
from typing import Optional

from rover_sim.control.command import CommandBus, Direction

from rover_hardware.backend import HardwareBackend, VehicleFeedback


@dataclass
class TankMixConfig:
    """
    Parameters for converting a single CommandBus into per-side commands.
    Mirrors SideBySideSkidSteerConfig in the simulator so the bench and
    real-hardware paths use the same mixing math.
    """

    track_width_m: float = 0.584
    max_speed_mps: float = 5.56  # MTT-154 20 km/h cap


def mix_differential(
    cmd: CommandBus,
    config: TankMixConfig,
) -> tuple[CommandBus, CommandBus]:
    """
    Turn a single high-level CommandBus into two per-MTT CommandBus instances.
    The center velocity is set by throttle (signed by direction) and the
    per-track differential is set by steer:

        v_center = sign(dir) * throttle * max_speed
        v_diff   = steer * max_speed
        v_left   = v_center - v_diff / 2
        v_right  = v_center + v_diff / 2

    Each side is renormalized back to a unit-throttle CommandBus with its own
    direction derived from the sign of the per-side velocity. Articulation
    steer is zero on each individual MTT — in tank mode the unit travels
    straight ahead and turning comes from the differential.
    """
    sign = 1.0 if cmd.direction == Direction.FORWARD else -1.0
    v_center = sign * cmd.throttle * config.max_speed_mps
    v_diff = cmd.steer * config.max_speed_mps

    v_left = v_center - v_diff / 2.0
    v_right = v_center + v_diff / 2.0

    return _per_side(v_left, cmd, config), _per_side(v_right, cmd, config)


def _per_side(v: float, cmd: CommandBus, config: TankMixConfig) -> CommandBus:
    magnitude = min(1.0, abs(v) / config.max_speed_mps)
    direction = Direction.FORWARD if v >= 0.0 else Direction.REVERSE
    return CommandBus(
        throttle=magnitude,
        brake=cmd.brake,
        steer=0.0,
        direction=direction,
        neutral=cmd.neutral,
        estop_handle=cmd.estop_handle,
        estop_machine=cmd.estop_machine,
        direction_mode=cmd.direction_mode,
        safety_unlocked=cmd.safety_unlocked,
        winch=cmd.winch,
        headlight=cmd.headlight,
    )


class TankCANBackend(HardwareBackend):
    """
    Drives two MTT-154s coupled side-by-side. Each MTT has its own CAN bus
    and its own backend instance; this class fans a single CommandBus out
    to both after applying the differential mixing. Feedback from the two
    units is averaged where it makes sense (speed) and kept per-side where
    it doesn't (articulation, faults).
    """

    def __init__(
        self,
        left: HardwareBackend,
        right: HardwareBackend,
        config: Optional[TankMixConfig] = None,
    ) -> None:
        self.left = left
        self.right = right
        self.config = config or TankMixConfig()

    def open(self) -> None:
        self.left.open()
        self.right.open()

    def close(self) -> None:
        self.left.close()
        self.right.close()

    def send(self, cmd: CommandBus) -> None:
        left_cmd, right_cmd = mix_differential(cmd, self.config)
        self.left.send(left_cmd)
        self.right.send(right_cmd)

    def read(self) -> Optional[VehicleFeedback]:
        lf = self.left.read()
        rf = self.right.read()
        if lf is None and rf is None:
            return None
        if lf is None:
            return rf
        if rf is None:
            return lf
        return VehicleFeedback(
            speed_mps=0.5 * (lf.speed_mps + rf.speed_mps),
            articulation_rad=0.0,
            battery_voltage_v=min(lf.battery_voltage_v, rf.battery_voltage_v),
            ecu_fault=lf.ecu_fault or rf.ecu_fault,
            raw={"left": lf.raw, "right": rf.raw},
        )
