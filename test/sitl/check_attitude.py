#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""L3 SITL attitude test (ADR-0018, spec 0004, spec 0012): the estimate against the truth.

Starts SITL with a motion profile and its UDP Link on a free port, and reads
vehicle_attitude, estimator_attitude, and vehicle_attitude_groundtruth through
the Link monitor library. Each estimate is compared with the truth carried to its
sample time along the true body rate. Fails when the tilt error (the angle between
the true and estimated down axes, that is roll and pitch together) exceeds 2 deg,
when the whole attitude error (which bounds the yaw error) exceeds 5 deg, when
no estimate arrives, or when a work item overran during the check.

With heading fusion on, checks the yaw within 3 deg of the truth after each profile;
without it, checks that the EKF's yaw drift is no worse than the Mahony filter's.
Checks that at least 95 % of the EKF's samples are within 3 standard deviations
(from its published variances) of the truth on each axis.

    check_attitude.py build/test/app/vehkernel-sitl --msg-dir build/test/generated/schema/python --motion rotate
"""

from __future__ import annotations

import argparse
import bisect
import math
import subprocess
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools" / "link"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from sitl_support import configure_persistent_params, free_udp_port  # noqa: E402
from vklink.monitor import LinkMonitor, UdpConnection, WorkItemOverruns, load_messages  # noqa: E402
from vklink.session import ParamType  # noqa: E402

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


# How much more than the Mahony filter's the EKF's yaw may drift over a profile without heading fusion.
# Both share most of the error (the same samples and their integration), so they differ by hundredths of
# a degree, either way from run to run; 0.5 deg still fails an EKF that loses its heading.
YAW_DRIFT_MARGIN_DEG = 0.5


def error_rotation_vector(truth_q, est_q) -> tuple[float, float, float]:
    """NED error rotation vector [rad] about North, East, Down (indices 0, 1, 2)."""
    q_err = multiply(truth_q, conjugate(est_q))
    if q_err[0] < 0:
        q_err = (-q_err[0], -q_err[1], -q_err[2], -q_err[3])
    angle = 2.0 * math.atan2(math.sqrt(q_err[1]**2 + q_err[2]**2 + q_err[3]**2), abs(q_err[0]))
    if angle < 1e-9:
        return (2.0 * q_err[1], 2.0 * q_err[2], 2.0 * q_err[3])
    scale = angle / math.sin(angle / 2.0)
    return (scale * q_err[1], scale * q_err[2], scale * q_err[3])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("sitl", help="the vehkernel-sitl executable")
    parser.add_argument("--msg-dir", required=True, help="directory of the generated vkmsg.py")
    parser.add_argument("--motion", required=True, choices=("still", "tilt", "rotate", "shake"))
    parser.add_argument("--max-tilt-deg", type=float, default=2.0)
    parser.add_argument("--max-error-deg", type=float, default=5.0)
    parser.add_argument("--primary", choices=("MAHONY", "EKF"), default=None,
                        help="primary attitude estimator (EST_PRIMARY)")
    parser.add_argument("--mag-type", choices=("AUTO", "OFF", "HEADING"), default=None,
                        help="magnetometer heading fusion mode (EKF_MAG_TYPE)")
    parser.add_argument("--expect-secondary", action="store_true",
                        help="require secondary estimator samples")
    args = parser.parse_args()

    port = free_udp_port()
    duration = PROFILE_END_S + 2 * MARGIN_S

    temp_dir = None
    param_file = None
    if args.primary is not None or args.mag_type is not None:
        temp_dir = tempfile.TemporaryDirectory()
        param_file = str(Path(temp_dir.name) / "params.bin")
        params_to_set = []
        if args.primary is not None:
            primary_val = 1 if args.primary == "MAHONY" else 2
            params_to_set.append(("EST_PRIMARY", ParamType.INT32, primary_val))
        if args.mag_type is not None:
            mag_val = {"AUTO": 0, "OFF": 1, "HEADING": 2}[args.mag_type]
            params_to_set.append(("EKF_MAG_TYPE", ParamType.INT32, mag_val))
        err = configure_persistent_params(args, param_file, params_to_set)
        if err is not None:
            print(f"FAIL: {err}", file=sys.stderr)
            return 1

    cmd = [args.sitl, "--duration", str(duration), "--udp", f"127.0.0.1:{port}",
           "--motion", args.motion]
    if param_file is not None:
        cmd.extend(["--param-file", param_file])

    sitl = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    print(sitl.stdout.readline(), end="")  # the link line

    truth_times: list[int] = []
    truths: list[tuple] = []
    primary_estimates: list[dict] = []
    secondary_estimates: list[dict] = []
    # Overruns during the check, without the host's stalls of the whole process.
    overruns = WorkItemOverruns()

    def record(name: str, instance: int, message) -> None:
        if name == "work_item_status":
            overruns.record(message)
            return
        if name == "vehicle_attitude_groundtruth" and instance == 0:
            truth_times.append(message.timestamp)
            truths.append((tuple(message.q), tuple(message.angular_velocity)))
        elif name == "vehicle_attitude" and instance == 0:
            primary_estimates.append({
                "estimator_id": message.estimator_id,
                "timestamp": message.timestamp_sample,
                "q": tuple(message.q),
                "variance": tuple(message.variance),
            })
        elif name == "estimator_attitude":
            secondary_estimates.append({
                "estimator_id": message.estimator_id,
                "instance": instance,
                "timestamp": message.timestamp_sample,
                "q": tuple(message.q),
                "variance": tuple(message.variance),
            })

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
    if temp_dir is not None:
        temp_dir.cleanup()

    if sitl.returncode != 0:
        failures.append(f"vehkernel-sitl exited with {sitl.returncode}\n{errors}")

    def evaluate_stream(estimates_list: list[dict]) -> tuple[float, float, int, list[tuple[dict, tuple, tuple[float, float, float]]]]:
        worst_tilt = worst_error = 0.0
        compared = 0
        matches = []
        for est in estimates_list:
            timestamp = est["timestamp"]
            i = bisect.bisect_left(truth_times, timestamp)
            candidates = [j for j in (i - 1, i) if 0 <= j < len(truths)]
            if not candidates:
                continue
            j = min(candidates, key=lambda k: abs(truth_times[k] - timestamp))
            q, rate = truths[j]
            dt = (timestamp - truth_times[j]) * 1e-6
            truth_q = multiply(q, from_rotation_vector(tuple(r * dt for r in rate)))
            worst_tilt = max(worst_tilt, tilt_between(truth_q, est["q"]))
            worst_error = max(worst_error, angle_between(truth_q, est["q"]))
            dtheta = error_rotation_vector(truth_q, est["q"])
            matches.append((est, truth_q, dtheta))
            compared += 1
        return worst_tilt, worst_error, compared, matches

    prim_tilt, prim_error, prim_compared, prim_matches = evaluate_stream(primary_estimates)
    sec_tilt, sec_error, sec_compared, sec_matches = evaluate_stream(secondary_estimates)

    print("\n".join(monitor.summary()))
    print(f"attitude_check motion={args.motion} primary_estimates={prim_compared} "
          f"primary_tilt={prim_tilt:.2f} deg primary_error={prim_error:.2f} deg")
    if sec_compared > 0:
        print(f"attitude_check secondary_estimates={sec_compared} "
              f"secondary_tilt={sec_tilt:.2f} deg secondary_error={sec_error:.2f} deg")

    expected_primary_id = 1 if args.primary == "MAHONY" else 2
    expected_secondary_id = 2 if expected_primary_id == 1 else 1
    expected_secondary_instance = 1 if expected_secondary_id == 2 else 0

    if primary_estimates:
        for est in primary_estimates:
            if est["estimator_id"] != expected_primary_id:
                failures.append(
                    f"primary stream has estimator_id {est['estimator_id']}, expected {expected_primary_id}"
                )
                break

    if secondary_estimates:
        for est in secondary_estimates:
            if est["estimator_id"] != expected_secondary_id or est["instance"] != expected_secondary_instance:
                failures.append(
                    f"secondary stream has estimator_id {est['estimator_id']} instance {est['instance']}, "
                    f"expected estimator_id {expected_secondary_id} instance {expected_secondary_instance}"
                )
                break

    if prim_compared < 100:
        failures.append(f"only {prim_compared} primary estimates could be compared with the truth")
    if prim_tilt > args.max_tilt_deg:
        failures.append(f"primary tilt error reached {prim_tilt:.2f} deg, more than {args.max_tilt_deg:g} deg")
    if prim_error > args.max_error_deg:
        failures.append(f"primary attitude error reached {prim_error:.2f} deg, more than {args.max_error_deg:g} deg")

    if args.expect_secondary and sec_compared < 100:
        failures.append(f"only {sec_compared} secondary estimates could be compared with the truth, expected at least 100")
    if sec_tilt > args.max_tilt_deg:
        failures.append(f"secondary tilt error reached {sec_tilt:.2f} deg, more than {args.max_tilt_deg:g} deg")
    if sec_error > args.max_error_deg:
        failures.append(f"secondary attitude error reached {sec_error:.2f} deg, more than {args.max_error_deg:g} deg")

    # Distinguish Mahony and EKF matches across primary and secondary streams.
    ekf_matches = [m for m in prim_matches if m[0]["estimator_id"] == 2] or \
                  [m for m in sec_matches if m[0]["estimator_id"] == 2]
    mahony_matches = [m for m in prim_matches if m[0]["estimator_id"] == 1] or \
                     [m for m in sec_matches if m[0]["estimator_id"] == 1]

    # EKF checks:
    if ekf_matches:
        # Check 1: 3-sigma check (at least 95 % of samples within 3 SD per axis).
        axis_3sd_counts = [0, 0, 0]
        for est, _, dtheta in ekf_matches:
            var = est["variance"]
            for ax in range(3):
                sd = math.sqrt(max(0.0, var[ax]))
                if abs(dtheta[ax]) <= 3.0 * sd:
                    axis_3sd_counts[ax] += 1
        total_ekf = len(ekf_matches)
        fractions = [axis_3sd_counts[ax] / total_ekf for ax in range(3)]
        axis_names = ("roll (North)", "pitch (East)", "yaw (Down)")
        for ax in range(3):
            if fractions[ax] < 0.95:
                failures.append(f"EKF {axis_names[ax]} axis had only {fractions[ax]*100:.1f} % of samples within 3 SD, "
                                f"expected at least 95 %")
        print(f"ekf_check 3sd_fractions: roll={fractions[0]*100:.1f}% pitch={fractions[1]*100:.1f}% "
              f"yaw={fractions[2]*100:.1f}%")

        # Check 2: Heading fusion yaw check or yaw drift check against Mahony.
        if args.mag_type == "HEADING":
            last_ekf_dtheta = ekf_matches[-1][2]
            ekf_final_yaw_err_deg = abs(math.degrees(last_ekf_dtheta[2]))
            if ekf_final_yaw_err_deg > 3.0:
                failures.append(f"with heading fusion, EKF final yaw error reached {ekf_final_yaw_err_deg:.2f} deg, "
                                f"more than 3.0 deg")
            else:
                print(f"ekf_heading_check final_yaw_err={ekf_final_yaw_err_deg:.2f} deg <= 3.0 deg")
        elif mahony_matches:
            common_time = min(ekf_matches[-1][0]["timestamp"], mahony_matches[-1][0]["timestamp"])
            covered_truth_times = [t for t in truth_times if t <= common_time]
            target_time = covered_truth_times[-1] if covered_truth_times else common_time
            ekf_common = min(ekf_matches, key=lambda m: abs(m[0]["timestamp"] - target_time))
            mahony_common = min(mahony_matches, key=lambda m: abs(m[0]["timestamp"] - target_time))
            ekf_final_yaw_err_deg = abs(math.degrees(ekf_common[2][2]))
            mahony_final_yaw_err_deg = abs(math.degrees(mahony_common[2][2]))
            # Both estimates compared at the last truth sample both cover (target_time). This is a
            # sanity bound, not a ranking: SITL shows whether the EKF keeps its heading as well as the
            # Mahony filter, while which one drifts less depends on tuning, compared on the board (#155).
            if ekf_final_yaw_err_deg > mahony_final_yaw_err_deg + YAW_DRIFT_MARGIN_DEG:
                failures.append(f"EKF yaw drift ({ekf_final_yaw_err_deg:.2f} deg) worse than Mahony "
                                f"({mahony_final_yaw_err_deg:.2f} deg)")
            else:
                print(f"ekf_yaw_drift_check ekf={ekf_final_yaw_err_deg:.2f} deg "
                      f"mahony={mahony_final_yaw_err_deg:.2f} deg")

    judged = overruns.judge()
    if judged is None:
        failures.append("the Link carried too little work item timing")
    else:
        for name, count in sorted(judged[0].items()):
            if count != 0:
                failures.append(f"work item {name} overran {count} times during the check")

    for failure in failures:
        print(f"FAIL: {failure}", file=sys.stderr)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
