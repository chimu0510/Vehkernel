#!/usr/bin/env python3
"""L3 SITL parameter test (ADR-0018, spec 0005): parameters over the Link.

First, with SITL at rest, the Link monitor's param command lists every
parameter with its metadata and a matching parameter set hash, reads and sets
one, sets SENS_BOARD_ROT by name and sees it pending reboot, and is refused,
with the reason, a value out of range and an unknown name. Then, with SITL under the shake profile and the accelerometer rejection
turned off through ATT_ACC_REJ, the estimate follows the linear acceleration
bursts barely at a small ATT_KP and far at a large one set in between: the
gain reaches the running filter. Last, values set over the Link are back
after SITL restarts with the same --param-file, parameter_status reporting
nothing skipped, and SITL without --param-file starts from the defaults.

    check_params.py build/test/app/vehkernel-sitl tools/link/link_monitor.py \\
        --msg-dir build/test/generated/schema/python --param-meta build/test/generated/params/params.json
"""

from __future__ import annotations

import argparse
import bisect
import subprocess
import sys
import tempfile
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools" / "link"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from check_attitude import free_udp_port, from_rotation_vector, multiply, tilt_between  # noqa: E402
from vklink.monitor import LinkMonitor, UdpConnection, WorkItemOverruns, load_messages, load_param_metadata  # noqa: E402
from vklink.session import ParamType, ParamValue  # noqa: E402

HANDSHAKE_S = 4.0
# The shake profile's bursts: 0.5 s every 1.5 s from 3 s to 10 s after boot.
SMALL_GAIN_WINDOW_S = (3.0, 5.5)
LARGE_GAIN_AT_S = 5.6
LARGE_GAIN_WINDOW_S = (6.0, 10.0)
SMALL_KP, LARGE_KP = 0.2, 8.0


def param_cli(args, address: str, *command: str) -> subprocess.CompletedProcess:
    result = subprocess.run(
        [sys.executable, args.monitor, "--udp", address, "--msg-dir", args.msg_dir, "--param-meta", args.param_meta,
         "--handshake-timeout", str(HANDSHAKE_S), "param", *command],
        capture_output=True, text=True, timeout=30, check=False)
    # Quoted, so the refusals the check expects do not read as its own failures.
    print(f"$ param {' '.join(command)}  (exit {result.returncode})")
    for line in (result.stdout + result.stderr).splitlines():
        print(f"| {line}")
    return result


def check_cli(args) -> list[str]:
    """The param command against SITL at rest."""
    failures = []
    address = f"127.0.0.1:{free_udp_port()}"
    sitl = subprocess.Popen([args.sitl, "--duration", "0", "--udp", address],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    print(sitl.stdout.readline(), end="")  # the link line
    try:
        metadata = load_param_metadata(args.param_meta)
        listed = param_cli(args, address, "list")
        names = [line.split()[1].removeprefix("name=") for line in listed.stdout.splitlines()]
        if listed.returncode != 0 or names != sorted(metadata.by_name):
            failures.append(f"param list returned {names}, expected {sorted(metadata.by_name)}")
        if "warning" in listed.stderr:
            failures.append("the parameter set hash does not match the build's metadata")
        if not all("description=" in line for line in listed.stdout.splitlines()):
            failures.append("param list shows parameters without their metadata")

        got = param_cli(args, address, "get", "ATT_KP")
        if got.returncode != 0 or " value=1 " not in got.stdout:
            failures.append("param get ATT_KP did not return the default 1")
        changed = param_cli(args, address, "set", "ATT_KP", "2.5")
        if changed.returncode != 0 or " value=2.5 " not in changed.stdout:
            failures.append("param set ATT_KP 2.5 did not take")
        got = param_cli(args, address, "get", "ATT_KP")
        if " value=2.5 " not in got.stdout:
            failures.append("param get ATT_KP did not return the new value 2.5")

        # A reboot_required parameter: stored, flagged until the next boot, and set by enum name.
        rotated = param_cli(args, address, "set", "SENS_BOARD_ROT", "ROLL_180")
        if rotated.returncode != 0 or "value_name=ROLL_180" not in rotated.stdout \
                or "pending_reboot=1" not in rotated.stdout:
            failures.append("param set SENS_BOARD_ROT ROLL_180 was not taken and flagged pending reboot")
        rotated = param_cli(args, address, "get", "SENS_BOARD_ROT")
        if "pending_reboot=1" not in rotated.stdout:
            failures.append("param get SENS_BOARD_ROT does not show the pending reboot")
        got = param_cli(args, address, "get", "ATT_KP")
        if "pending_reboot" in got.stdout:
            failures.append("ATT_KP, applied at once, shows a pending reboot")

        for command, reason in ((("set", "ATT_KP", "50"), "out of range"), (("set", "ATT_NONE", "1"), "unknown name"),
                                (("get", "ATT_NONE"), "unknown name")):
            refused = param_cli(args, address, *command)
            if refused.returncode != 1 or f"rejected: {reason}" not in refused.stderr:
                failures.append(f"param {' '.join(command)} was not refused as {reason}")
    finally:
        sitl.terminate()
        sitl.communicate(timeout=30)
    return failures


def check_gain(args) -> list[str]:
    """ATT_KP changed while the estimator runs under the shake profile."""
    failures = []
    port = free_udp_port()
    sitl = subprocess.Popen([args.sitl, "--duration", "14", "--udp", f"127.0.0.1:{port}", "--motion", "shake"],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    print(sitl.stdout.readline(), end="")

    truth_times: list[int] = []
    truths: list[tuple] = []
    estimates: list[tuple[int, tuple]] = []
    # Only overruns during the check count, and not a host stall's: SITL shares
    # the host, and stalls say nothing about parameters.
    overruns = WorkItemOverruns()

    def record(name: str, instance: int, message) -> None:
        if name == "work_item_status":
            overruns.record(message)
            return
        if instance != 0:
            return
        if name == "vehicle_attitude_groundtruth":
            truth_times.append(message.timestamp)
            truths.append((tuple(message.q), tuple(message.angular_velocity)))
        elif name == "vehicle_attitude":
            estimates.append((message.timestamp_sample, tuple(message.q)))

    def run_until(t_s: float) -> None:
        while not truth_times or truth_times[-1] < t_s * 1e6:
            monitor.poll(0.05)

    def set_float(name: str, value: float) -> None:
        reply = monitor.set_param(name, ParamType.FLOAT, value, 3.0)
        if not isinstance(reply, ParamValue):
            failures.append(f"setting {name} to {value:g} failed: {reply}")

    connection = UdpConnection("127.0.0.1", port)
    monitor = LinkMonitor(connection, load_messages(args.msg_dir))
    monitor.listeners.append(record)
    try:
        if not monitor.handshake(HANDSHAKE_S):
            failures.append("no handshake with SITL")
        else:
            set_float("ATT_ACC_REJ", 1.0)  # the bursts no longer count as acceleration
            set_float("ATT_KP", SMALL_KP)
            run_until(LARGE_GAIN_AT_S)
            set_float("ATT_KP", LARGE_KP)
            run_until(LARGE_GAIN_WINDOW_S[1])
    finally:
        connection.close()
    _, errors = sitl.communicate(timeout=60)
    if sitl.returncode != 0:
        failures.append(f"vehkernel-sitl exited with {sitl.returncode}\n{errors}")

    def worst_tilt(window: tuple[float, float]) -> tuple[float, int]:
        worst, compared = 0.0, 0
        for timestamp, estimate in estimates:
            if not window[0] * 1e6 <= timestamp < window[1] * 1e6:
                continue
            i = bisect.bisect_left(truth_times, timestamp)
            candidates = [j for j in (i - 1, i) if 0 <= j < len(truths)]
            if not candidates:
                continue
            j = min(candidates, key=lambda k: abs(truth_times[k] - timestamp))
            q, rate = truths[j]
            dt = (timestamp - truth_times[j]) * 1e-6
            truth = multiply(q, from_rotation_vector(tuple(r * dt for r in rate)))
            worst = max(worst, tilt_between(truth, estimate))
            compared += 1
        return worst, compared

    small, small_count = worst_tilt(SMALL_GAIN_WINDOW_S)
    large, large_count = worst_tilt(LARGE_GAIN_WINDOW_S)
    print(f"gain_check small_kp={SMALL_KP:g} max_tilt_deg={small:.2f} estimates={small_count} "
          f"large_kp={LARGE_KP:g} max_tilt_deg={large:.2f} estimates={large_count}")
    if min(small_count, large_count) < 100:
        failures.append("too few estimates to compare with the truth")
    elif not (large > 2 * small and large > 5.0):
        failures.append(f"ATT_KP {LARGE_KP:g} tilted the estimate {large:.2f} deg, not clearly more than "
                        f"{small:.2f} deg at ATT_KP {SMALL_KP:g}")
    judged = overruns.judge()
    if judged is None:
        failures.append("the Link carried too little work item timing")
    else:
        for name, count in sorted(judged[0].items()):
            if count != 0:
                failures.append(f"work item {name} overran {count} times during the check")
    return failures


class Sitl:
    """SITL with its UDP Link and the Link monitor library connected to it."""

    def __init__(self, args, *extra: str):
        port = free_udp_port()
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


def check_persistence(args) -> list[str]:
    """Values set over the Link, kept in --param-file across a restart."""
    failures = []
    with tempfile.TemporaryDirectory() as directory:
        param_file = str(Path(directory) / "params.bin")
        with Sitl(args, "--param-file", param_file) as sitl:
            if not sitl.monitor.handshake(HANDSHAKE_S):
                return ["no handshake with SITL"]
            for name, value in (("ATT_KP", 3.0), ("ATT_KI", 0.1)):
                reply = sitl.monitor.set_param(name, ParamType.FLOAT, value, 3.0)
                if not isinstance(reply, ParamValue):
                    failures.append(f"setting {name} failed: {reply}")
            status = None
            for _ in range(5):
                status = sitl.status()
                if status is not None and status.unsaved == 0:
                    break
            if status is None or status.unsaved != 0:
                failures.append(f"the changes were not stored: {status}")
            else:
                print(f"stored: bytes_used={status.bytes_used} sector={status.active_sector}")
        with Sitl(args, "--param-file", param_file) as sitl:
            if not sitl.monitor.handshake(HANDSHAKE_S):
                return failures + ["no handshake with SITL after the restart"]
            for name, expected in (("ATT_KP", 3.0), ("ATT_KI", 0.1)):
                reply = sitl.monitor.get_param(name, 3.0)
                if not isinstance(reply, ParamValue) or abs(reply.value - expected) > 1e-6:
                    failures.append(f"{name} after the restart is {getattr(reply, 'value', reply)}, "
                                    f"expected {expected:g}")
            status = sitl.status()
            skipped = None if status is None else (status.skipped_unknown + status.skipped_wrong_type +
                                                   status.skipped_out_of_range + status.skipped_corrupt)
            print(f"restarted: skipped={skipped} unsaved={getattr(status, 'unsaved', None)}")
            if skipped != 0 or status.unsaved != 0 or status.failed:
                failures.append(f"parameter_status after the restart: {status}")
    with Sitl(args) as sitl:
        if not sitl.monitor.handshake(HANDSHAKE_S):
            return failures + ["no handshake with SITL without --param-file"]
        reply = sitl.monitor.get_param("ATT_KP", 3.0)
        if not isinstance(reply, ParamValue) or reply.value != 1.0:
            failures.append(f"without --param-file ATT_KP is {getattr(reply, 'value', reply)}, not the default 1")
    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("sitl", help="the vehkernel-sitl executable")
    parser.add_argument("monitor", help="tools/link/link_monitor.py")
    parser.add_argument("--msg-dir", required=True, help="directory of the generated vkmsg.py")
    parser.add_argument("--param-meta", required=True, help="the build's parameter metadata file")
    args = parser.parse_args()

    failures = check_cli(args) + check_gain(args) + check_persistence(args)
    for failure in failures:
        print(f"FAIL: {failure}", file=sys.stderr)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
