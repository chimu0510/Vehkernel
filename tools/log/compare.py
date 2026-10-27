#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Compares recorded and replayed ULog files for faithfulness and estimation metrics (spec 0011, ADR-0018 L2).

Faithfulness comparison:
- Matches logged vehicle_attitude and vehicle_local_odometry samples with replayed
  samples by timestamp_sample.
- Fields left out of the faithfulness check:
  - `timestamp`: this is the publication time (the replay clock is set to the input's
    time, whereas the board's execution occurred later in real time).
  All other fields defined in the log's schema format are compared.
- For float fields: NaN on both sides is considered equal; NaN on one side only is
  a mismatch; difference is checked against --tolerance.
- Integer and boolean fields must match exactly regardless of --tolerance.
- A comparison matching no samples in the window fails.

Metrics comparison against ground truth:
- Aligns truth into the estimate's local level frame (origin at first compared sample,
  yaw = 0 at first compared sample).
- Uses bisect for O(log N) truth sample interpolation.
"""

from __future__ import annotations

import argparse
import bisect
import math
from pathlib import Path
import sys
from typing import Any

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools" / "log"))

import vkulog  # noqa: E402
from vkulog.reader import DataMessage  # noqa: E402

# The publication timestamp is the only field left out of faithfulness comparison.
# The replay clock is set to the input sample's timestamp; the board published later.
# Samples are matched by timestamp_sample; every other field is compared.
LEFT_OUT_FIELDS = {"timestamp"}


def multiply(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def conjugate(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    return (q[0], -q[1], -q[2], -q[3])


def angle_between(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> float:
    d = multiply(conjugate(a), b)
    return math.degrees(2 * math.atan2(math.sqrt(d[1] ** 2 + d[2] ** 2 + d[3] ** 2), abs(d[0])))


def yaw_of(q: tuple[float, float, float, float]) -> float:
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def wrap(angle: float) -> float:
    return math.remainder(angle, 2 * math.pi)


def slerp(q0: tuple[float, float, float, float], q1: tuple[float, float, float, float], f: float) -> tuple[float, float, float, float]:
    dot = sum(a * b for a, b in zip(q0, q1))
    if dot < 0.0:
        q1 = (-q1[0], -q1[1], -q1[2], -q1[3])
        dot = -dot
    if dot > 0.9995:
        res = tuple(a + (b - a) * f for a, b in zip(q0, q1))
    else:
        theta_0 = math.acos(max(-1.0, min(1.0, dot)))
        theta = theta_0 * f
        s0 = math.cos(theta) - dot * math.sin(theta) / math.sin(theta_0)
        s1 = math.sin(theta) / math.sin(theta_0)
        res = tuple(s0 * a + s1 * b for a, b in zip(q0, q1))
    norm = math.sqrt(sum(c * c for c in res))
    if norm < 1e-9:
        return (1.0, 0.0, 0.0, 0.0)
    return (res[0] / norm, res[1] / norm, res[2] / norm, res[3] / norm)


def interpolate_truth_attitude(gt_list: list[dict[str, Any]], target_us: int) -> tuple[float, float, float, float]:
    if not gt_list:
        return (1.0, 0.0, 0.0, 0.0)
    timestamps = [s["timestamp"] for s in gt_list]
    idx = bisect.bisect_left(timestamps, target_us)
    if idx == 0:
        return gt_list[0]["q"]
    if idx >= len(gt_list):
        return gt_list[-1]["q"]
    before = gt_list[idx - 1]
    after = gt_list[idx]
    if after["timestamp"] == before["timestamp"] or target_us == before["timestamp"]:
        return before["q"]
    f = (target_us - before["timestamp"]) / (after["timestamp"] - before["timestamp"])
    return slerp(before["q"], after["q"], f)


def interpolate_truth_odometry(gt_list: list[dict[str, Any]], target_us: int) -> tuple[tuple[float, float, float], float]:
    if not gt_list:
        return ((0.0, 0.0, 0.0), 0.0)
    timestamps = [s["timestamp"] for s in gt_list]
    idx = bisect.bisect_left(timestamps, target_us)
    if idx == 0:
        s0 = gt_list[0]
        return (s0["position"], yaw_of(s0["q"]))
    if idx >= len(gt_list):
        s_end = gt_list[-1]
        return (s_end["position"], yaw_of(s_end["q"]))
    before = gt_list[idx - 1]
    after = gt_list[idx]
    if after["timestamp"] == before["timestamp"] or target_us == before["timestamp"]:
        return (before["position"], yaw_of(before["q"]))
    f = (target_us - before["timestamp"]) / (after["timestamp"] - before["timestamp"])
    pos = (
        before["position"][0] + (after["position"][0] - before["position"][0]) * f,
        before["position"][1] + (after["position"][1] - before["position"][1]) * f,
        before["position"][2] + (after["position"][2] - before["position"][2]) * f,
    )
    yaw = yaw_of(before["q"]) + wrap(yaw_of(after["q"]) - yaw_of(before["q"])) * f
    return (pos, yaw)


def extract_log(path: str | Path) -> dict[str, list[dict[str, Any]]]:
    """Extract messages decoded generically from each message's format in the log."""
    reader = vkulog.ULogReader(path)
    data: dict[str, list[dict[str, Any]]] = {}

    for r in reader.records:
        if isinstance(r, DataMessage):
            sub = reader.subscriptions.get(r.msg_id)
            if sub is None:
                continue
            name = sub.message_name
            fmt = reader.formats.get(name)
            if fmt is None:
                continue
            decoded = fmt.decode(r.data)
            if name not in data:
                data[name] = []
            data[name].append(decoded)

    return data


def compare_field(
    field_name: str,
    rec_val: Any,
    rep_val: Any,
    tolerance: float,
    max_diffs: dict[str, float],
    failures: list[str],
) -> None:
    """Compare a single field or tuple of elements handling types and NaNs."""
    if isinstance(rec_val, (tuple, list)):
        if not isinstance(rep_val, (tuple, list)) or len(rec_val) != len(rep_val):
            failures.append(f"{field_name}: tuple shape mismatch ({rec_val!r} vs {rep_val!r})")
            return
        for i, (r_elem, p_elem) in enumerate(zip(rec_val, rep_val)):
            compare_field(f"{field_name}[{i}]", r_elem, p_elem, tolerance, max_diffs, failures)
        return

    # Integer or bool field: must match exactly regardless of tolerance
    if isinstance(rec_val, (int, bool)) and isinstance(rep_val, (int, bool)):
        if rec_val != rep_val:
            msg = f"{field_name}: integer/bool mismatch (recorded {rec_val} != replayed {rep_val})"
            if msg not in failures:
                failures.append(msg)
        return

    # Floating point comparison with NaN handling
    if isinstance(rec_val, float) and isinstance(rep_val, float):
        r_nan, p_nan = math.isnan(rec_val), math.isnan(rep_val)
        if r_nan and p_nan:
            # Both NaN: equal
            return
        if r_nan != p_nan:
            # One NaN, one not: mismatch
            msg = f"{field_name}: NaN mismatch (recorded {rec_val} != replayed {rep_val})"
            if msg not in failures:
                failures.append(msg)
            return
        diff = abs(rec_val - rep_val)
        max_diffs[field_name] = max(max_diffs.get(field_name, 0.0), diff)
        return

    if rec_val != rep_val:
        msg = f"{field_name}: value mismatch ({rec_val!r} != {rep_val!r})"
        if msg not in failures:
            failures.append(msg)


def compare_topic_samples(
    topic_name: str,
    rec_samples: dict[int, dict[str, Any]],
    rep_samples: dict[int, dict[str, Any]],
    tolerance: float,
    max_diffs: dict[str, float],
    failures: list[str],
) -> int:
    matched_count = 0
    for ts, rc in sorted(rec_samples.items()):
        if ts not in rep_samples:
            failures.append(f"{topic_name}: sample at timestamp_sample {ts} in recorded but not replayed")
            continue
        matched_count += 1
        rp = rep_samples[ts]
        for field_name, rec_val in rc.items():
            if field_name in LEFT_OUT_FIELDS:
                continue
            if field_name not in rp:
                failures.append(f"{topic_name}.{field_name}: field missing in replayed message")
                continue
            rep_val = rp[field_name]
            compare_field(f"{topic_name}.{field_name}", rec_val, rep_val, tolerance, max_diffs, failures)
    return matched_count


def compare_logs(
    recorded_path: str | Path,
    replayed_path: str | Path,
    tolerance: float = 0.0,
    truth_path: str | Path | None = None,
    reference_path: str | Path | None = None,
) -> bool:
    rec_data = extract_log(recorded_path)
    rep_data = extract_log(replayed_path)

    truth_att_gt: list[dict[str, Any]] = []
    truth_odom_gt: list[dict[str, Any]] = []

    if truth_path:
        t_data = extract_log(truth_path)
        truth_att_gt = t_data.get("vehicle_attitude_groundtruth", [])
        truth_odom_gt = t_data.get("vehicle_local_odometry_groundtruth", [])
    elif reference_path:
        ref_data = extract_log(reference_path)
        for a in ref_data.get("vehicle_attitude", []):
            truth_att_gt.append({"timestamp": a.get("timestamp_sample", a["timestamp"]), "q": a["q"]})
        for o in ref_data.get("vehicle_local_odometry", []):
            truth_odom_gt.append({
                "timestamp": o.get("timestamp_sample", o["timestamp"]),
                "position": o["position"],
                "q": o["q"],
            })
    else:
        truth_att_gt = rec_data.get("vehicle_attitude_groundtruth", [])
        truth_odom_gt = rec_data.get("vehicle_local_odometry_groundtruth", [])

    # Window from first estimator_restart if present
    restart_ts: int | None = None
    if "estimator_restart" in rec_data and rec_data["estimator_restart"]:
        restart_ts = rec_data["estimator_restart"][0]["timestamp"]

    all_rec_ts = [
        s["timestamp_sample"]
        for tname in ("vehicle_attitude", "vehicle_local_odometry")
        for s in rec_data.get(tname, [])
        if "timestamp_sample" in s
    ]
    min_ts = min(all_rec_ts) if all_rec_ts else 0
    max_ts = max(all_rec_ts) if all_rec_ts else 0

    if restart_ts is not None:
        start_ts = restart_ts
        duration_s = max(0, max_ts - restart_ts) * 1e-6
        print(f"Comparing window: from first estimator_restart at {restart_ts} us to {max_ts} us (duration {duration_s:.3f} s)")
    else:
        start_ts = min_ts
        duration_s = max(0, max_ts - min_ts) * 1e-6
        print(f"Comparing window: whole log from {min_ts} us to {max_ts} us (duration {duration_s:.3f} s)")

    print(f"Left out of faithfulness check: {', '.join(sorted(LEFT_OUT_FIELDS))} (publication time; matched by timestamp_sample)")

    # Filter to compared window
    rec_att = {
        s["timestamp_sample"]: s
        for s in rec_data.get("vehicle_attitude", [])
        if s.get("timestamp_sample", 0) >= start_ts
    }
    rep_att = {
        s["timestamp_sample"]: s
        for s in rep_data.get("vehicle_attitude", [])
        if s.get("timestamp_sample", 0) >= start_ts
    }

    rec_odom = {
        s["timestamp_sample"]: s
        for s in rec_data.get("vehicle_local_odometry", [])
        if s.get("timestamp_sample", 0) >= start_ts
    }
    rep_odom = {
        s["timestamp_sample"]: s
        for s in rep_data.get("vehicle_local_odometry", [])
        if s.get("timestamp_sample", 0) >= start_ts
    }

    failures: list[str] = []
    max_diffs: dict[str, float] = {}

    matched_att = compare_topic_samples("vehicle_attitude", rec_att, rep_att, tolerance, max_diffs, failures)
    matched_odom = compare_topic_samples("vehicle_local_odometry", rec_odom, rep_odom, tolerance, max_diffs, failures)

    total_matched = matched_att + matched_odom
    if total_matched == 0:
        print("Faithfulness: FAILED (no matching samples in compared window)")
        return False

    for field, diff in sorted(max_diffs.items()):
        if diff > tolerance:
            failures.append(f"{field}: max difference {diff:.6e} > tolerance {tolerance:.6e}")

    # Summary
    failed_fields = {f.split(":")[0].strip() for f in failures}
    overall_max_diff = max(max_diffs.values(), default=0.0)
    if not failures and overall_max_diff == 0.0:
        print("Faithfulness: exact")
    elif not failures:
        print(f"Faithfulness: within tolerance {tolerance}")
        for field, diff in sorted(max_diffs.items()):
            if diff > 0.0:
                print(f"  {field}: largest difference {diff:.6e}")
    else:
        print(f"Faithfulness: FAILED ({len(failures)} mismatches)")
        for fail in failures:
            print(f"  FAIL: {fail}")
        for field, diff in sorted(max_diffs.items()):
            if field not in failed_fields and diff > 0.0:
                print(f"  OK: {field}: max difference {diff:.6e}")

    # Estimation metrics against ground truth
    # Align truth to estimate's frame at start of window (ADR-0014, check_odometry.py)
    if (truth_att_gt or truth_odom_gt) and (rep_att or rep_odom):
        first_rep_ts = min([ts for ts in (list(rep_att.keys()) + list(rep_odom.keys())) if ts >= start_ts], default=start_ts)

        # Baseline truth orientation and position at restart
        truth_pos_start = (0.0, 0.0, 0.0)
        truth_yaw_start = 0.0

        if truth_odom_gt:
            truth_pos_start, truth_yaw_start = interpolate_truth_odometry(truth_odom_gt, first_rep_ts)
        elif truth_att_gt:
            truth_q_start = interpolate_truth_attitude(truth_att_gt, first_rep_ts)
            truth_yaw_start = yaw_of(truth_q_start)

        if truth_att_gt:
            att_errors: list[float] = []
            q_align_yaw = (
                math.cos(-truth_yaw_start / 2.0),
                0.0,
                0.0,
                math.sin(-truth_yaw_start / 2.0),
            )
            for ts, sample in sorted(rep_att.items()):
                q_truth_world = interpolate_truth_attitude(truth_att_gt, ts)
                # Rotate truth world attitude by -truth_yaw_start about Z so initial yaw aligns with 0
                q_truth_aligned = multiply(q_align_yaw, q_truth_world)
                att_errors.append(angle_between(sample["q"], q_truth_aligned))
            if att_errors:
                mean_att = sum(att_errors) / len(att_errors)
                max_att = max(att_errors)
                end_att = att_errors[-1]
                print(f"Metrics: attitude error mean={mean_att:.4f} deg, max={max_att:.4f} deg, end={end_att:.4f} deg")

        if truth_odom_gt and rep_odom:
            last_ts = max(rep_odom.keys())
            last_sample = rep_odom[last_ts]
            truth_pos_world, truth_yaw_world = interpolate_truth_odometry(truth_odom_gt, last_ts)

            # Displacement in world coordinates relative to start
            dx_w = truth_pos_world[0] - truth_pos_start[0]
            dy_w = truth_pos_world[1] - truth_pos_start[1]
            dz_w = truth_pos_world[2] - truth_pos_start[2]

            # Rotate displacement into estimate frame (initial heading along X)
            cos_y = math.cos(truth_yaw_start)
            sin_y = math.sin(truth_yaw_start)
            x_aligned = cos_y * dx_w + sin_y * dy_w
            y_aligned = -sin_y * dx_w + cos_y * dy_w
            truth_pos_aligned = (x_aligned, y_aligned, dz_w)

            truth_yaw_aligned = wrap(truth_yaw_world - truth_yaw_start)

            pos_error = math.hypot(
                last_sample["position"][0] - truth_pos_aligned[0],
                last_sample["position"][1] - truth_pos_aligned[1],
            )
            heading_error_deg = math.degrees(abs(wrap(yaw_of(last_sample["q"]) - truth_yaw_aligned)))
            print(f"Metrics: position error at end={pos_error:.4f} m, heading error at end={heading_error_deg:.4f} deg")

    return len(failures) == 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("recorded", help="path to recorded log file")
    parser.add_argument("replayed", help="path to replayed log file")
    parser.add_argument("--tolerance", type=float, default=0.0, help="maximum allowable difference per field")
    parser.add_argument("--truth", help="optional path to truth log file")
    parser.add_argument("--reference", help="optional path to reference log file")
    args = parser.parse_args()

    ok = compare_logs(
        recorded_path=args.recorded,
        replayed_path=args.replayed,
        tolerance=args.tolerance,
        truth_path=args.truth,
        reference_path=args.reference,
    )
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
