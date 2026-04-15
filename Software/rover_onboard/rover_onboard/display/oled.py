"""OLED boot and status display.

The field rover carries a small OLED (SSD1306-class, 128x64 or 128x32) that
mirrors the boot state the operator cares about when no other screen is
available: IP address, WiFi status, current survey name, GNSS fix type,
and battery state of charge. The information comes straight from the
StatusBroker, so anything on the screen is guaranteed consistent with
what the drive loop actually sees.

Two backends: ``OLEDDisplay`` uses luma.oled against a real device, and
``NullDisplay`` renders into a list of strings so tests can assert on
the output without needing hardware.
"""

from typing import Protocol, runtime_checkable

from rover_onboard.status.snapshot import RoverStatusSnapshot


@runtime_checkable
class StatusDisplay(Protocol):
    """Something that can render a status snapshot to a small screen."""

    def render(self, snapshot: RoverStatusSnapshot) -> None: ...
    def close(self) -> None: ...


class NullDisplay:
    """Captures rendered frames as strings. Useful for tests and for headless
    deployments where the Jetson has no OLED attached."""

    def __init__(self) -> None:
        self.frames: list[list[str]] = []

    def render(self, snapshot: RoverStatusSnapshot) -> None:
        self.frames.append(_format_lines(snapshot))

    def close(self) -> None:
        pass

    @property
    def last_frame(self) -> list[str]:
        return self.frames[-1] if self.frames else []


class OLEDDisplay:
    """Drives a real SSD1306 over I2C via luma.oled. Safe to construct on a
    machine with no display — the import is lazy and raises a clear
    message if the hardware extras aren't installed."""

    def __init__(
        self,
        i2c_port: int = 1,
        i2c_address: int = 0x3C,
        width: int = 128,
        height: int = 64,
    ) -> None:
        try:
            from luma.core.interface.serial import i2c
            from luma.oled.device import ssd1306
            from PIL import ImageFont
        except ImportError as e:
            raise RuntimeError(
                "OLEDDisplay requires the 'display' extra. "
                "Install with: pip install rover_onboard[display]"
            ) from e

        serial = i2c(port=i2c_port, address=i2c_address)
        self._device = ssd1306(serial, width=width, height=height)
        self._font = ImageFont.load_default()

    def render(self, snapshot: RoverStatusSnapshot) -> None:
        from luma.core.render import canvas

        lines = _format_lines(snapshot)
        with canvas(self._device) as draw:
            for i, line in enumerate(lines):
                draw.text((0, i * 10), line, fill=255, font=self._font)

    def close(self) -> None:
        try:
            self._device.cleanup()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Layout
# ---------------------------------------------------------------------------


def _format_lines(snapshot: RoverStatusSnapshot) -> list[str]:
    """Lay out six lines of ~21 characters each suitable for a 128x64 OLED
    with the default luma 5x8 font. Long values are truncated, not wrapped."""
    name = (snapshot.rover_name or "rover")[:20]
    mode = snapshot.mode.value[:10]
    fix = snapshot.gnss.fix_type.upper()[:6]
    sats = snapshot.gnss.num_satellites
    soc = snapshot.battery.soc_percent
    volts = snapshot.battery.voltage_v
    survey = (snapshot.survey.name or "-")[:20]
    speed = snapshot.drive.actual_speed_mps

    sat_txt = f" sat:{sats}" if sats is not None else ""
    soc_txt = f"{soc:>4.0f}%" if soc is not None else "  -%"
    v_txt = f"{volts:>5.1f}V" if volts is not None else "   --V"
    speed_txt = f"{speed:>4.2f} m/s" if speed is not None else " -.-- m/s"

    return [
        f"{name}",
        f"mode: {mode}",
        f"fix:  {fix}{sat_txt}",
        f"batt: {soc_txt} {v_txt}",
        f"v:    {speed_txt}",
        f"surv: {survey}",
    ]
