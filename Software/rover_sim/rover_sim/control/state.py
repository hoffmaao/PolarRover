from dataclasses import dataclass
from enum import Enum


class FixType(Enum):
    """
    GNSS fix quality ladder. Modeled after the u-blox ZED-F9P dual-band RTK
    receiver, which is the Nisse's primary position sensor. Ordered from
    worst to best for comparison purposes.
    """

    NONE = "none"     # no fix — position unknown / stale
    SPP = "spp"       # standalone positioning,  ~1–2 m horizontal
    DGPS = "dgps"     # DGPS / SBAS corrected,    ~0.5–1 m
    FLOAT = "float"   # RTK float,                 ~10–30 cm
    FIXED = "fixed"   # RTK fixed (integer ambig), ~1–2 cm


@dataclass(frozen=True)
class RoverState:
    """
    The state a Driver sees from its sensors at a given instant. This is what
    the sim publishes after each step (via the Sensors layer) and what the
    real hardware's sensor stack would expose through the same interface.

    Frame: local ENU (East-North-Up), meters. Heading in radians, measured
    counter-clockwise from East (standard math convention); 0 = east, pi/2 = north.
    Speed is signed: negative means reverse motion along the heading.

    `fix_type` and `position_std_m` expose the GNSS quality the driver must
    tolerate. Drive code that only works at FIXED precision will behave badly
    in the field; the sim surfaces degradation explicitly so nav stacks are
    forced to handle FLOAT / DGPS / SPP / dropout states.
    """

    t: float = 0.0
    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0
    speed: float = 0.0
    yaw_rate: float = 0.0
    fix_type: FixType = FixType.FIXED
    position_std_m: float = 0.0
