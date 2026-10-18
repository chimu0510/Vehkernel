#!/usr/bin/env python3
"""L3 SITL attitude test (ADR-0018, spec 0004): the estimate against the truth.

Starts SITL with a motion profile and its UDP Link on a free port, and reads
vehicle_attitude and vehicle_attitude_groundtruth through the Link monitor
library. Each estimate is compared with the truth carried to its sample time
along the true body rate. Fails when the tilt error (the angle between the
true and estimated down axes, that is roll and pitch together) exceeds 2 deg,
when the whole attitude error (which bounds the yaw error) exceeds 5 deg, when
no estimate arrives, or when a work item overran during the check.

Angles between attitudes are used rather than differences of roll, pitch and
yaw, which jump near pitch +-90 deg, where the rotate profile goes.

    check_attitude.py build/test/app/vehkernel-sitl --msg-dir build/test/generated/schema/python --motion rotate
"""

from __future__ import annotations

import argparse
import bisect
import math
import socket
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools" / "link"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from check_report import parse_report  # noqa: E402
from vklink.monitor import LinkMonitor, UdpConnection, load_messages  # noqa: E402

# Every profile's motion ends by 10 s after boot.
PROFILE_END_S = 10.0
MARGIN_S = 2.0


def multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw)


def conjugate(q):
    return (q[0], -q[1], -q[2], -q[3])


def from_rotation_vector(v):
    angle = math.sqrt(sum(c * c for c in v))
    if angle < 1e-9:
        return (1.0, v[0] / 2, v[1] / 2, v[2] / 2)
    s = math.sin(angle / 2) / angle
    return (math.cos(angle / 2), v[0] * s, v[1] * s, v[2] * s)


def angle_between(a, b) -> float:
    """Degrees of the rotation from attitude a to attitude b."""
    d = multiply(conjugate(a), b)
    return math.degrees(2 * math.atan2(math.sqrt(d[1] ** 2 + d[2] ** 2 + d[3] ** 2), abs(d[0])))


def down_axis(q):
    """The level frame's down axis seen in the body frame."""
    w, x, y, z = q
    return (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y))


def tilt_between(a, b) -> float:
    da, db = down_axis(a), down_axis(b)
    dot = max(-1.0, min(1.0, sum(p * q for p, q in zip(da, db))))
    return math.degrees(math.acos(dot))


def free_udp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("sitl", help="the vehkernel-sitl executable")
    parser.add_argument("--msg-dir", required=True, help="directory of the generated vkmsg.py")
    parser.add_argument("--motion", required=True, choices=("still", "tilt", "rotate", "shake"))
    parser.add_argument("--max-tilt-deg", type=float, default=2.0)
    parser.add_argument("--max-error-deg", type=float, default=5.0)
    args = parser.parse_args()

    port = free_udp_port()
    duration = PROFILE_END_S + 2 * MARGIN_S
    sitl = subprocess.Popen([args.sitl, "--duration", str(duration), "--udp", f"127.0.0.1:{port}",
                             "--motion", args.motion], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    print(sitl.stdout.readline(), end="")  # the link line

    truth_times: list[int] = []
    truths: list[tuple] = []
    estimates: list[tuple[int, tuple]] = []

    def record(name: str, instance: int, message) -> None:
        if instance != 0:
            return
        if name == "vehicle_attitude_groundtruth":
            truth_times.append(message.timestamp)
            truths.append((tuple(message.q), tuple(message.angular_velocity)))
        elif name == "vehicle_attitude":
            estimates.append((message.timestamp_sample, tuple(message.q)))

    connection = UdpConnection("127.0.0.1", port)
    monitor = LinkMonitor(connection, load_messages(args.msg_dir))
    monitor.listeners.append(record)
    failures = []
    try:
        if not monitor.handshake(MARGIN_S):
            failures.append("no handshake with SITL")
        else:
            monitor.run(PROFILE_END_S + MARGIN_S / 2)
    finally:
        connection.close()
    _, errors = sitl.communicate(timeout=duration + 30)
    if sitl.returncode != 0:
        failures.append(f"vehkernel-sitl exited with {sitl.returncode}\n{errors}")

    worst_tilt = worst_error = 0.0
    compared = 0
    for timestamp, estimate in estimates:
        i = bisect.bisect_left(truth_times, timestamp)
        candidates = [j for j in (i - 1, i) if 0 <= j < len(truths)]
        if not candidates:
            continue
        j = min(candidates, key=lambda k: abs(truth_times[k] - timestamp))
        q, rate = truths[j]
        dt = (timestamp - truth_times[j]) * 1e-6
        truth = multiply(q, from_rotation_vector(tuple(r * dt for r in rate)))
        worst_tilt = max(worst_tilt, tilt_between(truth, estimate))
        worst_error = max(worst_error, angle_between(truth, estimate))
        compared += 1

    print("\n".join(monitor.summary()))
    print(f"attitude_check motion={args.motion} estimates={compared} max_tilt_error_deg={worst_tilt:.2f} "
          f"max_error_deg={worst_error:.2f}")
    if compared < 100:
        failures.append(f"only {compared} estimates could be compared with the truth")
    if worst_tilt > args.max_tilt_deg:
        failures.append(f"tilt error reached {worst_tilt:.2f} deg, more than {args.max_tilt_deg:g} deg")
    if worst_error > args.max_error_deg:
        failures.append(f"attitude error reached {worst_error:.2f} deg, more than {args.max_error_deg:g} deg")
    _, reported = parse_report("\n".join(monitor.summary()))
    for item in reported:
        if int(item["overruns"]) != 0:
            failures.append(f"work item {item['name']} overran {item['overruns']} times during the check")

    for failure in failures:
        print(f"FAIL: {failure}", file=sys.stderr)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
