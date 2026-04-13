import pytest

from rover_sim.control import CommandBus, Direction
from rover_sim.safety import SafetyConfig, SafetyFilter
from rover_sim.vehicle.base import VehicleState


def _moving(speed: float, direction: Direction = Direction.FORWARD) -> VehicleState:
    return VehicleState(speed=speed)


def _stopped() -> VehicleState:
    return VehicleState(speed=0.0)


def test_fr_change_rejected_while_moving():
    f = SafetyFilter(SafetyConfig(f_r_stop_eps_mps=0.05))
    cmd = CommandBus(throttle=1.0, direction=Direction.FORWARD)

    out = f.filter(cmd, _moving(2.0), dt=0.01)
    assert out.direction is Direction.FORWARD

    switch = CommandBus(throttle=1.0, direction=Direction.REVERSE)
    out = f.filter(switch, _moving(2.0), dt=0.01)
    assert out.direction is Direction.FORWARD  # held — vehicle still moving


def test_fr_change_accepted_once_stopped():
    f = SafetyFilter(SafetyConfig(f_r_stop_eps_mps=0.05))
    f.filter(CommandBus(throttle=1.0, direction=Direction.FORWARD), _moving(2.0), 0.01)
    f.filter(CommandBus(brake=1.0, direction=Direction.FORWARD), _stopped(), 0.01)

    out = f.filter(
        CommandBus(throttle=1.0, direction=Direction.REVERSE), _stopped(), 0.01
    )
    assert out.direction is Direction.REVERSE


def test_learning_mode_caps_throttle():
    cfg = SafetyConfig(
        max_speed_mps=5.56, learning_max_speed_mps=1.11, learning_mode=True
    )
    f = SafetyFilter(cfg)
    out = f.filter(CommandBus(throttle=1.0), _stopped(), dt=0.01)
    assert out.throttle == pytest.approx(1.11 / 5.56, rel=1e-9)


def test_learning_mode_inactive_passes_full_throttle():
    cfg = SafetyConfig(learning_mode=False)
    f = SafetyFilter(cfg)
    out = f.filter(CommandBus(throttle=1.0), _stopped(), dt=0.01)
    assert out.throttle == 1.0


def test_auto_neutral_trips_after_timeout_and_forces_throttle_zero():
    cfg = SafetyConfig(auto_neutral_timeout_s=1.0)
    f = SafetyFilter(cfg)

    # 1.5 seconds of zero input
    for _ in range(150):
        f.filter(CommandBus(), _stopped(), dt=0.01)

    assert f.neutral_tripped
    # Now the driver tries to command throttle — should be locked out
    out = f.filter(CommandBus(throttle=1.0), _stopped(), dt=0.01)
    assert out.throttle == 0.0


def test_auto_neutral_not_tripped_while_brake_in_use():
    cfg = SafetyConfig(auto_neutral_timeout_s=1.0)
    f = SafetyFilter(cfg)

    for _ in range(500):
        f.filter(CommandBus(brake=0.5), _stopped(), dt=0.01)
    assert not f.neutral_tripped


def test_auto_neutral_cleared_by_explicit_neutral_command():
    cfg = SafetyConfig(auto_neutral_timeout_s=1.0)
    f = SafetyFilter(cfg)

    for _ in range(150):
        f.filter(CommandBus(), _stopped(), dt=0.01)
    assert f.neutral_tripped

    # Driver cycles the neutral button (single frame with neutral=True)
    f.filter(CommandBus(neutral=True), _stopped(), dt=0.01)
    assert not f.neutral_tripped

    # Subsequent throttle is accepted
    out = f.filter(CommandBus(throttle=0.5), _stopped(), dt=0.01)
    assert out.throttle == 0.5


def test_reset_restores_defaults():
    cfg = SafetyConfig(auto_neutral_timeout_s=1.0)
    f = SafetyFilter(cfg)

    for _ in range(150):
        f.filter(CommandBus(), _stopped(), dt=0.01)
    assert f.neutral_tripped
    f.reset()
    assert not f.neutral_tripped
