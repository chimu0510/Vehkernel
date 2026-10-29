#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""L3 SITL magnetometer test (spec 0012): sample rate, heading against truth, read sizes, no overruns, and --mag-off.

Starts SITL with the rotate motion profile and its UDP Link, and reads
magnetometer_sample and vehicle_attitude_groundtruth through the Link monitor
library. Judges the magnetometer rate at 100 Hz within 5 %, the levelled heading
within 1 deg of truth after the first second, I2C read transfer lengths, no
work item overruns, and a second run with --mag-off publishing no magnetometer samples.

A magnetometer sample's timestamp is the time of the driver run that read it
from the ICM20948 FIFO. The AK09916 sampled the magnetic field earlier: at most
one AK09916 period (10 ms) plus one driver read period (5 ms) before the FIFO read.
The check compares each sample against ground truth across this [timestamp - 15 ms,
timestamp] window, taking the smallest heading error.

    check_magnetometer.py build/test/app/vehkernel-sitl --msg-dir build/test/generated/schema/python
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

from vklink.monitor import LinkMonitor, UdpConnection, WorkItemOverruns, load_messages  # noqa: E402

MARGIN_S = 2.0


def multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def from_rotation_vector(v):
    angle = math.sqrt(sum(c * c for c in v))
    if angle < 1e-9:
        return (1.0, v[0] / 2, v[1] / 2, v[2] / 2)
    s = math.sin(angle / 2) / angle
    return (math.cos(angle / 2), v[0] * s, v[1] * s, v[2] * s)


def euler_from_quaternion(q):
    w, x, y, z = q
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    if abs(sin_pitch) > 0.99999:
        roll = 0.0
        pitch = math.copysign(math.pi / 2.0, sin_pitch)
        yaw = 2.0 * math.atan2(z, w)
    else:
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(sin_pitch)
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    cr = math.cos(roll / 2.0)
    sr = math.sin(roll / 2.0)
    cp = math.cos(pitch / 2.0)
    sp = math.sin(pitch / 2.0)
    cy = math.cos(yaw / 2.0)
    sy = math.sin(yaw / 2.0)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def rotate_vector(q, v):
    w, x, y, z = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    ux_tx = y * tz - z * ty
    ux_ty = z * tx - x * tz
    ux_tz = x * ty - y * tx
    return (
        vx + w * tx + ux_tx,
        vy + w * ty + ux_ty,
        vz + w * tz + ux_tz,
    )


def wrap_pi(angle: float) -> float:
    return math.remainder(angle, 2.0 * math.pi)


def heading_from_field(roll: float, pitch: float, field: tuple[float, float, float]) -> float:
    level_q = quaternion_from_euler(roll, pitch, 0.0)
    levelled = rotate_vector(level_q, field)
    return wrap_pi(math.atan2(-levelled[1], levelled[0]))


def free_udp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


def parse_report(text: str) -> tuple[dict[tuple[str, int], dict[str, str]], list[dict[str, str]], list[dict[str, str]]]:
    topics: dict[tuple[str, int], dict[str, str]] = {}
    work_items: list[dict[str, str]] = []
    i2c_reads: list[dict[str, str]] = []
    for line in text.splitlines():
        if not line.strip():
            continue
        parts = line.split()
        kind = parts[0]
        pairs = parts[1:]
        fields = dict(pair.split("=", 1) for pair in pairs if "=" in pair)
        if kind == "topic":
            topics[(fields["name"], int(fields["instance"]))] = fields
        elif kind == "work_item":
            work_items.append(fields)
        elif kind == "i2c_read":
            i2c_reads.append(fields)
    return topics, work_items, i2c_reads


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("sitl", help="the vehkernel-sitl executable")
    parser.add_argument("--msg-dir", required=True, help="directory of the generated vkmsg.py")
    args = parser.parse_args()

    # --- Run 1: Normal run with rotate motion profile ---
    port = free_udp_port()
    duration = 12.0
    sitl = subprocess.Popen([args.sitl, "--duration", str(duration), "--udp", f"127.0.0.1:{port}",
                             "--motion", "rotate"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    print(sitl.stdout.readline(), end="")  # the link line

    truth_times: list[int] = []
    truths: list[tuple] = []
    mag_samples: list[tuple[int, tuple[float, float, float]]] = []
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
        elif name == "magnetometer_sample":
            mag_samples.append((message.timestamp, tuple(message.field)))

    connection = UdpConnection("127.0.0.1", port)
    monitor = LinkMonitor(connection, load_messages(args.msg_dir))
    monitor.listeners.append(record)
    failures: list[str] = []
    try:
        if not monitor.handshake(MARGIN_S):
            failures.append("no handshake with SITL")
        else:
            monitor.run(duration - MARGIN_S)
    finally:
        connection.close()

    out, errors = sitl.communicate(timeout=duration + 30)
    if sitl.returncode != 0:
        failures.append(f"vehkernel-sitl exited with {sitl.returncode}\n{errors}")

    topics, _, i2c_reads = parse_report(out)

    # 1. Check magnetometer_sample rate from topic line in report
    mag_topic = topics.get(("magnetometer_sample", 0))
    if not mag_topic:
        failures.append("topic magnetometer_sample instance 0 not in SITL report")
    else:
        rate_hz = float(mag_topic["rate_hz"])
        if not (95.0 <= rate_hz <= 105.0):
            failures.append(f"magnetometer_sample rate {rate_hz:.2f} Hz not within 5 % of 100 Hz")

    # 2. Check I2C read transfer lengths
    seen_registers: set[int] = set()
    for r in i2c_reads:
        if r.get("device") != "icm20948":
            continue
        reg = int(r["register"], 16)
        length = int(r["length"])
        count = int(r["count"])
        seen_registers.add(reg)
        if reg == 0x2D:
            if length != 14:
                failures.append(f"ACCEL_XOUT_H (0x2d) read with unexpected length {length} (count {count})")
        elif reg == 0x70:
            if length != 2:
                failures.append(f"FIFO_COUNTH (0x70) read with unexpected length {length} (count {count})")
        elif reg == 0x72:
            if length not in (9, 18, 27, 36):
                failures.append(f"FIFO_R_W (0x72) read with unexpected length {length} (count {count})")
        else:
            if length != 1:
                failures.append(f"register {hex(reg)} read with unexpected length {length} (count {count})")

    if 0x2D not in seen_registers:
        failures.append("no read from ACCEL_XOUT_H (0x2d) recorded")
    if 0x70 not in seen_registers:
        failures.append("no read from FIFO_COUNTH (0x70) recorded")
    if 0x72 not in seen_registers:
        failures.append("no read from FIFO_R_W (0x72) recorded")

    # 3. Check work item overruns, setting host stalls aside as the other checks do (#170)
    judged = overruns.judge()
    if judged is None:
        failures.append("the Link carried too little work item timing")
    else:
        for name, count in sorted(judged[0].items()):
            if count != 0:
                failures.append(f"work item {name} overran {count} times during the check")

    # 4. Check heading error within 1 deg after first second
    worst_heading_error = 0.0
    compared = 0
    first_sample_time = mag_samples[0][0] if mag_samples else 0

    for timestamp, field in mag_samples:
        if timestamp < first_sample_time + 1_000_000:
            continue
        sample_min_error = float("inf")
        for offset_us in range(0, 15001, 250):
            t_eval = timestamp - offset_us
            i = bisect.bisect_left(truth_times, t_eval)
            candidates = [k for k in (i - 1, i) if 0 <= k < len(truths)]
            if not candidates:
                continue
            j = min(candidates, key=lambda k: abs(truth_times[k] - t_eval))
            q, rate = truths[j]
            dt = (t_eval - truth_times[j]) * 1e-6
            truth_q = multiply(q, from_rotation_vector(tuple(r * dt for r in rate)))
            true_roll, true_pitch, true_yaw = euler_from_quaternion(truth_q)
            mag_heading = heading_from_field(true_roll, true_pitch, field)
            error_deg = math.degrees(abs(wrap_pi(mag_heading - true_yaw)))
            if error_deg < sample_min_error:
                sample_min_error = error_deg
        if sample_min_error == float("inf"):
            continue
        worst_heading_error = max(worst_heading_error, sample_min_error)
        compared += 1
        if sample_min_error > 1.0:
            failures.append(f"sample at {timestamp} us had heading error {sample_min_error:.3f} deg > 1.0 deg")
            break

    if compared < 100:
        failures.append(f"only {compared} magnetometer samples could be compared with truth")

    print(f"magnetometer_check compared={compared} max_heading_error_deg={worst_heading_error:.2f}")

    # --- Run 2: Second run with --mag-off ---
    port2 = free_udp_port()
    sitl_off = subprocess.Popen([args.sitl, "--duration", "2.0", "--udp", f"127.0.0.1:{port2}",
                                 "--mag-off"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    out_off, err_off = sitl_off.communicate(timeout=30)
    if sitl_off.returncode != 0:
        failures.append(f"vehkernel-sitl with --mag-off exited with {sitl_off.returncode}\n{err_off}")
    else:
        topics_off, _, _ = parse_report(out_off)
        mag_off_topic = topics_off.get(("magnetometer_sample", 0))
        if mag_off_topic:
            pubs = int(mag_off_topic.get("publications", 0))
            rate = float(mag_off_topic.get("rate_hz", 0.0))
            if pubs != 0 or rate != 0.0:
                failures.append(f"--mag-off published magnetometer_sample: {pubs} publications, {rate:.2f} Hz")
        imu_off_topic = topics_off.get(("imu_sample", 0))
        if not imu_off_topic:
            failures.append("imu_sample not in --mag-off report")
        else:
            imu_rate = float(imu_off_topic.get("rate_hz", 0.0))
            if not (190.0 <= imu_rate <= 210.0):
                failures.append(f"--mag-off imu_sample rate {imu_rate:.2f} Hz not at 200 Hz within 5 %")

    for failure in failures:
        print(f"FAIL: {failure}", file=sys.stderr)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
