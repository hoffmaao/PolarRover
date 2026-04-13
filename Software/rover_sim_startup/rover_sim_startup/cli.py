"""CLI for `rover-sim-startup`."""

import socket
import sys
from pathlib import Path
from typing import Optional


def main(argv: Optional[list[str]] = None) -> int:
    argv = argv if argv is not None else sys.argv[1:]

    host = "0.0.0.0"
    port = 8000
    scenarios_dir: Optional[Path] = None

    i = 0
    while i < len(argv):
        arg = argv[i]
        if arg in {"-h", "--help", "help"}:
            _print_help()
            return 0
        if arg == "--host" and i + 1 < len(argv):
            host = argv[i + 1]
            i += 2
            continue
        if arg == "--port" and i + 1 < len(argv):
            port = int(argv[i + 1])
            i += 2
            continue
        if arg == "--scenarios-dir" and i + 1 < len(argv):
            scenarios_dir = Path(argv[i + 1])
            i += 2
            continue
        print(f"unknown argument: {arg}", file=sys.stderr)
        _print_help()
        return 2

    import uvicorn

    from rover_sim_startup.main import create_app

    app = create_app(scenarios_dir=scenarios_dir)

    local_ip = _get_local_ip()
    print()
    print("  PolarRover Mission Startup")
    print(f"  Local:   http://localhost:{port}")
    if host == "0.0.0.0" and local_ip:
        print(f"  Network: http://{local_ip}:{port}")
    print()

    uvicorn.run(app, host=host, port=port, log_level="warning")
    return 0


def _get_local_ip() -> Optional[str]:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return None


def _print_help() -> None:
    print("rover-sim-startup — PolarRover mission authoring web app")
    print()
    print("Usage: rover-sim-startup [options]")
    print()
    print("Options:")
    print("  --host HOST            bind address (default: 0.0.0.0)")
    print("  --port PORT            port (default: 8000)")
    print("  --scenarios-dir DIR    scenario storage directory")
    print("  --help                 show this help")
    print()
    print("Open http://localhost:8000 in a browser after starting.")
    print("Any device on the same LAN can access via your machine's IP.")


if __name__ == "__main__":
    raise SystemExit(main())
