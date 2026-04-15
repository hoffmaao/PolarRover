"""WiFi access-point fallback configuration.

Out in the field there's no infrastructure WiFi, so the rover falls back
to advertising its own access point with a known SSID and password. The
operator joins from a laptop or phone and hits http://<ip>:5000/ for
video + state. The upstream implementation we're porting runs hostapd
directly; here we only own the *configuration* and delegate the actual
AP bring-up to whichever system service (NetworkManager, hostapd,
AccessPopup) is installed on the Jetson.

This module's job is to produce a HotspotConfig that system-level code
can read and apply — not to run iptables/hostapd itself. That separation
makes the package installable on a laptop for testing without needing
root or WiFi privileges.
"""

import shutil
from dataclasses import dataclass


@dataclass(frozen=True)
class HotspotConfig:
    """Per-rover access-point settings. These should be unique per rover
    so operators know which one they're joining."""

    ssid: str
    password: str
    channel: int = 6
    country: str = "US"

    @classmethod
    def for_rover(cls, rover_name: str, password: str) -> "HotspotConfig":
        return cls(ssid=rover_name, password=password)


def hotspot_available() -> bool:
    """Best-effort check for whether a hotspot manager is installed on
    this system. Returns False on a laptop without NetworkManager, True
    on a Jetson with either NetworkManager or hostapd available."""
    return shutil.which("nmcli") is not None or shutil.which("hostapd") is not None
