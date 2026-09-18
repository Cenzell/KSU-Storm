"""Shared mDNS/DNS-SD helpers so the robot and driver station can find each
other without hardcoded IP addresses.

Uses python-zeroconf (a pure-Python mDNS implementation) rather than relying
on OS-level ".local" hostname resolution: that support is inconsistent
across driver-station laptops (Windows in particular doesn't resolve mDNS
names out of the box without Bonjour installed), while zeroconf implements
the protocol itself and works the same on every platform.

Both sides treat this as a purely additive convenience, never a dependency:
- ServiceAdvertiser (robot side) failing to start must not stop the robot
  from running — the driver station always has the static ROBOT_ADDRESSES
  list in comm.py as a fallback.
- ServiceDiscoverer (driver-station side) failing to start must not stop
  the driver station from connecting via the static address list.

Callers are expected to wrap construction in try/except and log a warning
rather than propagate — this module doesn't do that itself so failures
aren't silently swallowed here too.
"""

from __future__ import annotations

import logging
import socket
from typing import Callable, Optional

from zeroconf import ServiceInfo, ServiceStateChange, Zeroconf
from zeroconf import ServiceBrowser

logger = logging.getLogger(__name__)

SERVICE_TYPE = "_ksustorm._tcp.local."


def local_ipv4() -> str:
    """Best-effort local IPv4 address to advertise. Sends no traffic."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # UDP connect() doesn't send a packet; it just picks a local route.
        sock.connect(("8.8.8.8", 80))
        return sock.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        sock.close()


class ServiceAdvertiser:
    """Advertises this robot on the LAN via mDNS/DNS-SD. Robot-side only."""

    def __init__(self, instance_name: str, command_port: int, telemetry_port: int) -> None:
        self.instance_name = instance_name
        address = local_ipv4()
        self._zc = Zeroconf()
        self._info = ServiceInfo(
            SERVICE_TYPE,
            f"{instance_name}.{SERVICE_TYPE}",
            addresses=[socket.inet_aton(address)],
            port=command_port,
            properties={"telemetry_port": str(telemetry_port)},
        )
        self._zc.register_service(self._info)
        logger.info("mDNS: advertising %s at %s:%d (telemetry %d)",
                    instance_name, address, command_port, telemetry_port)

    def close(self) -> None:
        try:
            self._zc.unregister_service(self._info)
        except Exception:
            pass
        try:
            self._zc.close()
        except Exception:
            pass


# (name, address_or_None, command_port, telemetry_port) — address is None on removal.
DiscoveryCallback = Callable[[str, Optional[str], int, int], None]


class ServiceDiscoverer:
    """Discovers KSU-Storm robots on the LAN via mDNS/DNS-SD. Driver-station side."""

    def __init__(self, on_change: DiscoveryCallback) -> None:
        self._on_change = on_change
        self._zc = Zeroconf()
        self._browser = ServiceBrowser(self._zc, SERVICE_TYPE, handlers=[self._handle])

    def _handle(self, zeroconf: Zeroconf, service_type: str, name: str,
                state_change: ServiceStateChange) -> None:
        if state_change is ServiceStateChange.Removed:
            self._on_change(name, None, 0, 0)
            return

        try:
            info = zeroconf.get_service_info(service_type, name, timeout=1000)
        except Exception as exc:
            logger.debug("mDNS: get_service_info failed for %s: %s", name, exc)
            return

        if info is None or not info.addresses:
            return

        address = socket.inet_ntoa(info.addresses[0])
        telemetry_port = int(info.properties.get(b"telemetry_port", b"0") or 0)
        self._on_change(name, address, info.port, telemetry_port)

    def close(self) -> None:
        try:
            self._zc.close()
        except Exception:
            pass
