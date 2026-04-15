import heapq
import random
from dataclasses import dataclass
from typing import Any, Optional


@dataclass
class RadioLinkConfig:
    """
    Parameterized radio link model. Default profile is **WiFi LAN**, matching
    the Nisse CMP topology where a stationary central base station
    (sitting at the common midpoint of a CMP shot) broadcasts RTK corrections
    and coordination messages to both rovers over 2.4 / 5 GHz WiFi.

    Alternative profiles are available via classmethod factories for sub-GHz
    radios in case we ever need longer range or a peer-to-peer fallback:

        RadioLinkConfig.wifi_lan()    # default — WiFi LAN in CMP star topology
        RadioLinkConfig.xbee_900()    # Digi XBee 3 Pro 900 HP, P2P, sub-GHz
        RadioLinkConfig.lora_sf7()    # LoRa SX1262 at SF7, long range, low rate

    Each factory accepts keyword overrides so a scenario can tune any single
    parameter without copying the whole profile.
    """

    latency_ms: float = 3.0
    jitter_ms: float = 1.0
    loss_rate: float = 0.001
    max_payload_bytes: int = 1472  # UDP payload on standard 1500 B MTU
    min_interval_ms: float = 0.0   # WiFi MAC handles its own pacing
    seed: Optional[int] = None

    @classmethod
    def wifi_lan(cls, **overrides: Any) -> "RadioLinkConfig":
        base = dict(
            latency_ms=3.0,
            jitter_ms=1.0,
            loss_rate=0.001,
            max_payload_bytes=1472,
            min_interval_ms=0.0,
        )
        base.update(overrides)
        return cls(**base)

    @classmethod
    def xbee_900(cls, **overrides: Any) -> "RadioLinkConfig":
        base = dict(
            latency_ms=100.0,
            jitter_ms=30.0,
            loss_rate=0.02,
            max_payload_bytes=256,
            min_interval_ms=10.0,
        )
        base.update(overrides)
        return cls(**base)

    @classmethod
    def lora_sf7(cls, **overrides: Any) -> "RadioLinkConfig":
        base = dict(
            latency_ms=80.0,
            jitter_ms=20.0,
            loss_rate=0.03,
            max_payload_bytes=64,
            min_interval_ms=50.0,
        )
        base.update(overrides)
        return cls(**base)


class RadioLink:
    """
    Symmetric in-process radio link between exactly two endpoints. Models
    latency, jitter, random loss, a per-direction send rate limit, and a hard
    max payload size. Does NOT model RF physics, range, or terrain — those can
    be layered on later via pre-send hooks if needed.

    For the Nisse CMP star topology, instantiate two RadioLink objects
    side by side: one for base↔rover_A and one for base↔rover_B. The linked
    CMP controller (Phase 6) runs on the base station and treats each link
    independently. If we ever need true one-to-many broadcast semantics, a
    dedicated BroadcastLink can be added later; the two-link pattern is
    sufficient for Phase 2 and most test scenarios.

    Usage is side-labeled (`a` and `b`) so the linked-CMP controller can keep
    its two peers straight:

        link = RadioLink(RadioLinkConfig.wifi_lan())
        link.send(side="a", payload=..., t=now)
        incoming = link.poll(side="b", t=now)   # returns list[payload]

    Sends that are rate-limited, oversize, or lost in transit return False.
    Returned packets arrive in delivery-time order regardless of send order,
    which matches how a real radio handles jitter.
    """

    def __init__(self, config: Optional[RadioLinkConfig] = None) -> None:
        self.config = config or RadioLinkConfig()
        self._rng = random.Random(self.config.seed)
        self._queues: dict[str, list[tuple[float, int, Any]]] = {"a": [], "b": []}
        self._last_tx: dict[str, float] = {"a": -float("inf"), "b": -float("inf")}
        self._seq: dict[str, int] = {"a": 0, "b": 0}

    def reset(self) -> None:
        self._queues = {"a": [], "b": []}
        self._last_tx = {"a": -float("inf"), "b": -float("inf")}
        self._seq = {"a": 0, "b": 0}
        self._rng = random.Random(self.config.seed)

    def send(self, side: str, payload: Any, t: float, size_bytes: int = 0) -> bool:
        """Send `payload` from `side` (a or b) at time `t`. Returns True if TX'd."""
        if side not in ("a", "b"):
            raise ValueError(f"side must be 'a' or 'b', got {side!r}")
        cfg = self.config
        if size_bytes > cfg.max_payload_bytes:
            return False
        last = self._last_tx[side]
        if (t - last) * 1000.0 < cfg.min_interval_ms - 1e-9:
            return False

        self._last_tx[side] = t

        if self._rng.random() < cfg.loss_rate:
            return False

        jitter = self._rng.uniform(-cfg.jitter_ms, cfg.jitter_ms)
        deliver_t = t + (cfg.latency_ms + jitter) / 1000.0
        receiving_side = "b" if side == "a" else "a"
        seq = self._seq[side]
        self._seq[side] = seq + 1
        heapq.heappush(self._queues[receiving_side], (deliver_t, seq, payload))
        return True

    def poll(self, side: str, t: float) -> list[Any]:
        """Return all payloads whose delivery time has elapsed by `t`, in order."""
        if side not in ("a", "b"):
            raise ValueError(f"side must be 'a' or 'b', got {side!r}")
        queue = self._queues[side]
        out: list[Any] = []
        while queue and queue[0][0] <= t + 1e-12:
            _, _, payload = heapq.heappop(queue)
            out.append(payload)
        return out

    def pending(self, side: str) -> int:
        """Number of packets in flight toward `side` (for test assertions)."""
        return len(self._queues[side])
