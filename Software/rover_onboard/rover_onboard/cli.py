"""rover-onboard CLI.

Starts the onboard platform services — status broker, OLED display
(if hardware present), audio event dispatcher, and Flask/FastAPI web UI —
and keeps them running until killed. Designed to run as a systemd
service on boot; on a laptop it runs fine with --no-display --no-audio
for UI development.
"""

import argparse
import sys
import threading
import time
from typing import Optional

from rover_onboard.audio.events import events_from_state_change
from rover_onboard.audio.phrasebook import phrase_for
from rover_onboard.status.broker import StatusBroker
from rover_onboard.status.snapshot import Mode, RoverStatusSnapshot


def main(argv: Optional[list[str]] = None) -> int:
    args = _build_parser().parse_args(argv)

    broker = StatusBroker(
        initial=RoverStatusSnapshot(
            rover_name=args.rover_name, mode=Mode.IDLE, timestamp=time.time()
        )
    )

    if not args.no_audio:
        _wire_audio(broker, args)
    if not args.no_display:
        _wire_display(broker, args)
    if not args.no_web:
        _wire_web(broker, args)

    print(f"rover-onboard: running as {args.rover_name} (Ctrl-C to stop)", flush=True)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("rover-onboard: shutting down", flush=True)
    return 0


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="rover-onboard")
    p.add_argument("--rover-name", default="rover")
    p.add_argument("--no-audio", action="store_true", help="Disable TTS output.")
    p.add_argument("--no-display", action="store_true", help="Disable OLED output.")
    p.add_argument("--no-web", action="store_true", help="Disable the web UI server.")
    p.add_argument("--web-host", default="0.0.0.0")
    p.add_argument("--web-port", type=int, default=5000)
    return p


def _wire_audio(broker: StatusBroker, args) -> None:
    try:
        from rover_onboard.audio.tts import PyttsxTTS
        tts = PyttsxTTS()
    except Exception as e:
        print(f"rover-onboard: audio disabled ({e})", file=sys.stderr)
        return

    def _on_change(prev, curr) -> None:
        for event in events_from_state_change(prev, curr):
            tts.say(phrase_for(event))

    broker.subscribe(_on_change)


def _wire_display(broker: StatusBroker, args) -> None:
    try:
        from rover_onboard.display.oled import OLEDDisplay
        display = OLEDDisplay()
    except Exception as e:
        print(f"rover-onboard: OLED disabled ({e})", file=sys.stderr)
        return

    def _render(prev, curr) -> None:
        display.render(curr)

    broker.subscribe(_render)


def _wire_web(broker: StatusBroker, args) -> None:
    try:
        import uvicorn
        from rover_onboard.web.app import create_app
    except ImportError as e:
        print(f"rover-onboard: web UI disabled ({e})", file=sys.stderr)
        return

    app = create_app(broker)

    def _serve() -> None:
        uvicorn.run(app, host=args.web_host, port=args.web_port, log_level="warning")

    t = threading.Thread(target=_serve, daemon=True, name="rover-onboard-web")
    t.start()


if __name__ == "__main__":
    raise SystemExit(main())
