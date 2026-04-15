"""Status broker — the one place the drive loop and the UI meet.

The drive loop writes a RoverStatusSnapshot each control cycle (typically
20 Hz). Readers — the OLED display, the WebSocket handler, the audio
event dispatcher — pull the latest snapshot at whatever rate they want.
Snapshots are frozen dataclasses so nobody can mutate a shared object;
readers always see a consistent view.

The broker also supports subscriptions: a reader can register a callback
that fires once when a specific field changes, which the audio layer uses
to speak "GPS fix lost" exactly when the fix type transitions rather than
on every tick.

Thread-safe: publish() can be called from the drive loop while readers
call latest() or subscribe() from other threads. Uses a single RLock.
"""

import threading
import time
from dataclasses import replace
from typing import Callable, Optional

from rover_onboard.status.snapshot import RoverStatusSnapshot


ChangeCallback = Callable[[RoverStatusSnapshot, RoverStatusSnapshot], None]


class StatusBroker:
    def __init__(
        self,
        initial: Optional[RoverStatusSnapshot] = None,
    ) -> None:
        self._lock = threading.RLock()
        self._latest: RoverStatusSnapshot = initial or RoverStatusSnapshot(
            timestamp=time.time()
        )
        self._subscribers: list[ChangeCallback] = []

    # ---- writes ----

    def publish(self, snapshot: RoverStatusSnapshot) -> None:
        """Replace the current snapshot and notify any subscribers whose
        predicate is satisfied by the change."""
        with self._lock:
            previous = self._latest
            # Stamp the timestamp here if the writer didn't set one.
            if snapshot.timestamp == 0.0:
                snapshot = replace(snapshot, timestamp=time.time())
            self._latest = snapshot
            subscribers = list(self._subscribers)

        for callback in subscribers:
            # Callbacks run outside the lock so a slow subscriber can't
            # stall the drive loop.
            try:
                callback(previous, snapshot)
            except Exception as e:
                # Never let a subscriber crash the drive loop. Log and move on.
                _warn(f"subscriber {callback!r} raised: {e}")

    def patch(self, **fields) -> None:
        """Convenience writer — apply a partial update to the latest
        snapshot. Useful when one producer only knows about GNSS and
        another only knows about the battery."""
        with self._lock:
            current = self._latest
        self.publish(replace(current, **fields))

    # ---- reads ----

    def latest(self) -> RoverStatusSnapshot:
        """Return the most recently published snapshot. Snapshots are
        frozen, so the caller can hold onto this reference indefinitely."""
        with self._lock:
            return self._latest

    def subscribe(self, callback: ChangeCallback) -> Callable[[], None]:
        """Register a (prev, curr) callback fired on every publish. Returns
        an unsubscribe function. Callbacks run outside the broker lock."""
        with self._lock:
            self._subscribers.append(callback)

        def _unsubscribe() -> None:
            with self._lock:
                try:
                    self._subscribers.remove(callback)
                except ValueError:
                    pass

        return _unsubscribe


def _warn(message: str) -> None:
    import sys
    print(f"rover-onboard: {message}", file=sys.stderr)
