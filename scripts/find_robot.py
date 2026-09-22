#!/usr/bin/env python3
"""Print the mDNS hostname (or IP, as fallback) of a running KSU-Storm robot.

Used by scripts/deploy.sh to auto-target whichever Pi is currently running
robot.py, instead of a hardcoded hostname. This only finds a robot that is
*currently running* robot.py (that's when it advertises itself — see
lib/mdns.py) — for a brand new Pi that's never run the code yet, there's
nothing to discover; pass an explicit host/IP for that first deploy.

Usage:
    python3 scripts/find_robot.py [--timeout SECONDS]

Prints the discovered hostname (e.g. "raspberrypi.local") to stdout and
exits 0. If no hostname is available, prints the IP address instead. Exits
1 with nothing on stdout if no robot is found within the timeout.
"""

from __future__ import annotations

import argparse
import sys
import threading
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "lib"))

from mdns import ServiceDiscoverer  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=3.0,
                         help="seconds to wait for a robot to respond (default: 3.0)")
    args = parser.parse_args()

    found = threading.Event()
    result: dict = {}

    def on_change(name: str, address, command_port: int, telemetry_port: int, hostname) -> None:
        if address is None or found.is_set():
            return
        result["target"] = hostname or address
        found.set()

    try:
        discoverer = ServiceDiscoverer(on_change=on_change)
    except Exception as exc:
        print(f"mDNS discovery unavailable: {exc}", file=sys.stderr)
        return 1

    try:
        ok = found.wait(timeout=args.timeout)
    finally:
        discoverer.close()

    if not ok:
        print("No KSU-Storm robot found via mDNS (is robot.py running on it?)", file=sys.stderr)
        return 1

    print(result["target"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
