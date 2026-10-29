# SPDX-License-Identifier: GPL-3.0-or-later
"""Shared helpers for SITL L3 tests: free UDP port, SITL runner, and persistent parameters."""

from __future__ import annotations

from pathlib import Path
import socket
import subprocess
import sys
import time
from typing import Sequence

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools" / "link"))

from vklink.monitor import LinkMonitor, UdpConnection, load_messages  # noqa: E402
from vklink.session import ParamType, ParamValue  # noqa: E402


def free_udp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


class Sitl:
    """SITL with its UDP Link and the Link monitor library connected to it."""

    def __init__(self, args, *extra: str):
        port = free_udp_port()
        self.port = port
        self.process = subprocess.Popen([args.sitl, "--duration", "0", "--udp", f"127.0.0.1:{port}", *extra],
                                        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        print(self.process.stdout.readline(), end="")
        self.connection = UdpConnection("127.0.0.1", port)
        self.monitor = LinkMonitor(self.connection, load_messages(args.msg_dir))

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.connection.close()
        self.process.terminate()
        self.process.communicate(timeout=30)

    def status(self, timeout_s: float = 3.0):
        """The next parameter_status to arrive."""
        stats = self.monitor.topics.get(("parameter_status", 0))
        count = stats.messages if stats else 0
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.monitor.poll(0.05)
            stats = self.monitor.topics.get(("parameter_status", 0))
            if stats and stats.messages > count and stats.latest is not None:
                return stats.latest
        return None


def configure_persistent_params(
    args,
    param_file: str | Path,
    params: Sequence[tuple[str, ParamType, int | float]],
    timeout_s: float = 3.0,
) -> str | None:
    """Sets persistent parameters via SITL and verifies storage. Returns error message on failure, None on success."""
    with Sitl(args, "--param-file", str(param_file)) as sitl_setup:
        if not sitl_setup.monitor.handshake(timeout_s=5.0):
            return "no handshake with SITL during parameter setup"
        for name, ptype, val in params:
            reply = sitl_setup.monitor.set_param(name, ptype, val, timeout_s=timeout_s)
            if not isinstance(reply, ParamValue):
                return f"could not set {name}: {reply}"
        status = None
        for _ in range(10):
            status = sitl_setup.status()
            if status is not None and status.unsaved == 0:
                break
        if status is None or status.unsaved != 0:
            return f"parameter changes were not stored: {status}"
    return None
