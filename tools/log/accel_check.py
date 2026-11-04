#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Check vehicle board orientation (SENS_BOARD_ROT) from recorded ULogs (#188).

Reads imu_sample:0 and logged SENS_BOARD_ROT parameter from two .ulg files
recorded with the vehicle stationary in two poses: level, and nose tilted down
by at least 30 deg. Verifies that SENS_BOARD_ROT matches how the board sits
in the vehicle, or identifies the correct rotation from the 20 supported
board orientations.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys
from typing import Sequence

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools" / "log"))
sys.path.insert(0, str(REPO / "tools" / "link"))

import vkulog  # noqa: E402
from vkulog.reader import DataMessage  # noqa: E402

# Standard gravity in m/s^2.
GRAVITY = 9.80665

# Minimum duration between first and last sample kept in seconds.
MIN_VALID_S = 5.0

# Maximum standard deviation per axis for stationary vehicle.
MAX_ACCEL_STD = 0.15  # m/s^2
MAX_GYRO_STD = 0.02   # rad/s

# Maximum allowable error from standard gravity in mean magnitude.
MAX_MAGNITUDE_ERROR = 1.0  # m/s^2

# Minimum angle between level and nose-down poses in degrees.
MIN_POSE_ANGLE_DEG = 30.0

# Maximum allowable fit error for best candidate rotation in degrees.
MAX_FIT_DEG = 15.0


class CheckRefused(Exception):
    """Raised when a log cannot be checked or violates validity thresholds."""


@dataclass(frozen=True)
class RotationInfo:
    name: str
    value: int
    matrix: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]


def _rotation_case_vector(name: str, v: tuple[float, float, float]) -> tuple[float, float, float]:
    """Rotates a sensor-frame vector into the body frame per hal/include/vk/hal/rotation.hpp."""
    h = 0.7071067811865476  # sin(45 deg) = cos(45 deg)
    x, y, z = v
    if name == "NONE":
        return (x, y, z)
    if name == "YAW_45":
        return (h * (x - y), h * (x + y), z)
    if name == "YAW_90":
        return (-y, x, z)
    if name == "YAW_135":
        return (-h * (x + y), h * (x - y), z)
    if name == "YAW_180":
        return (-x, -y, z)
    if name == "YAW_225":
        return (h * (y - x), -h * (x + y), z)
    if name == "YAW_270":
        return (y, -x, z)
    if name == "YAW_315":
        return (h * (x + y), h * (y - x), z)
    if name == "ROLL_180":
        return (x, -y, -z)
    if name == "ROLL_180_YAW_45":
        return (h * (x + y), h * (x - y), -z)
    if name == "ROLL_180_YAW_90":
        return (y, x, -z)
    if name == "ROLL_180_YAW_135":
        return (h * (y - x), h * (y + x), -z)
    if name == "PITCH_180":
        return (-x, y, -z)
    if name == "ROLL_180_YAW_225":
        return (-h * (x + y), h * (y - x), -z)
    if name == "ROLL_180_YAW_270":
        return (-y, -x, -z)
    if name == "ROLL_180_YAW_315":
        return (h * (x - y), -h * (x + y), -z)
    if name == "ROLL_90":
        return (x, -z, y)
    if name == "ROLL_270":
        return (x, z, -y)
    if name == "PITCH_90":
        return (z, y, -x)
    if name == "PITCH_270":
        return (-z, y, x)
    raise ValueError(f"Unknown rotation {name}")


def _make_matrix(name: str) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    c0 = _rotation_case_vector(name, (1.0, 0.0, 0.0))
    c1 = _rotation_case_vector(name, (0.0, 1.0, 0.0))
    c2 = _rotation_case_vector(name, (0.0, 0.0, 1.0))
    return (
        (c0[0], c1[0], c2[0]),
        (c0[1], c1[1], c2[1]),
        (c0[2], c1[2], c2[2]),
    )


# The 20 rotations declared in drivers/params.toml and hal/include/vk/hal/rotation.hpp.
ROTATION_SPECS: list[tuple[str, int]] = [
    ("NONE", 0),
    ("YAW_45", 1),
    ("YAW_90", 2),
    ("YAW_135", 3),
    ("YAW_180", 4),
    ("YAW_225", 5),
    ("YAW_270", 6),
    ("YAW_315", 7),
    ("ROLL_180", 8),
    ("ROLL_180_YAW_45", 9),
    ("ROLL_180_YAW_90", 10),
    ("ROLL_180_YAW_135", 11),
    ("PITCH_180", 12),
    ("ROLL_180_YAW_225", 13),
    ("ROLL_180_YAW_270", 14),
    ("ROLL_180_YAW_315", 15),
    ("ROLL_90", 16),
    ("ROLL_270", 20),
    ("PITCH_90", 24),
    ("PITCH_270", 25),
]

ROTATIONS: list[RotationInfo] = [
    RotationInfo(name=name, value=val, matrix=_make_matrix(name))
    for name, val in ROTATION_SPECS
]
ROTATIONS_BY_VALUE: dict[int, RotationInfo] = {r.value: r for r in ROTATIONS}
ROTATIONS_BY_NAME: dict[str, RotationInfo] = {r.name: r for r in ROTATIONS}


def mat_mul_vec(
    m: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
    v: tuple[float, float, float],
) -> tuple[float, float, float]:
    return (
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    )


def mat_transpose(
    m: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    return (
        (m[0][0], m[1][0], m[2][0]),
        (m[0][1], m[1][1], m[2][1]),
        (m[0][2], m[1][2], m[2][2]),
    )


def mat_mul_mat(
    a: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
    b: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]],
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    return (
        (
            a[0][0] * b[0][0] + a[0][1] * b[1][0] + a[0][2] * b[2][0],
            a[0][0] * b[0][1] + a[0][1] * b[1][1] + a[0][2] * b[2][1],
            a[0][0] * b[0][2] + a[0][1] * b[1][2] + a[0][2] * b[2][2],
        ),
        (
            a[1][0] * b[0][0] + a[1][1] * b[1][0] + a[1][2] * b[2][0],
            a[1][0] * b[0][1] + a[1][1] * b[1][1] + a[1][2] * b[2][1],
            a[1][0] * b[0][2] + a[1][1] * b[1][2] + a[1][2] * b[2][2],
        ),
        (
            a[2][0] * b[0][0] + a[2][1] * b[1][0] + a[2][2] * b[2][0],
            a[2][0] * b[0][1] + a[2][1] * b[1][1] + a[2][2] * b[2][1],
            a[2][0] * b[0][2] + a[2][1] * b[1][2] + a[2][2] * b[2][2],
        ),
    )


@dataclass(frozen=True)
class PoseSummary:
    name: str
    rotation: RotationInfo
    samples: int
    dropouts: int
    mean_accel: tuple[float, float, float]
    accel_std: tuple[float, float, float]
    max_accel_std: float
    mean_gyro: tuple[float, float, float]
    gyro_std: tuple[float, float, float]
    max_gyro_std: float
    mean_temp: float
    duration_s: float


@dataclass(frozen=True)
class CandidateScore:
    rotation: RotationInfo
    level_err_deg: float
    nose_err_deg: float
    fit_deg: float


def read_pose_log(log_path: str | Path, pose_name: str) -> PoseSummary:
    """Reads a pose log, verifies parameter integrity and stationary limits, and computes statistics."""
    try:
        reader = vkulog.ULogReader(log_path)
    except Exception as err:
        raise OSError(f"Could not open ULog file {log_path}: {err}") from err

    if "SENS_BOARD_ROT" not in reader.parameters:
        raise CheckRefused(f"Missing SENS_BOARD_ROT parameter in {log_path}")

    raw_rot = reader.parameters["SENS_BOARD_ROT"]
    rot_info: RotationInfo | None = None
    if isinstance(raw_rot, int):
        rot_info = ROTATIONS_BY_VALUE.get(raw_rot)
    elif isinstance(raw_rot, str):
        rot_info = ROTATIONS_BY_NAME.get(raw_rot)
        if rot_info is None:
            try:
                rot_info = ROTATIONS_BY_VALUE.get(int(raw_rot))
            except ValueError:
                pass

    if rot_info is None:
        raise CheckRefused(
            f"Invalid SENS_BOARD_ROT value {raw_rot!r} in {log_path}; must be one of the 20 supported rotations"
        )

    for change in reader.parameter_changes:
        if change.name == "SENS_BOARD_ROT":
            raise CheckRefused(f"Parameter SENS_BOARD_ROT changed during {log_path}")

    all_accel: list[tuple[float, float, float]] = []
    all_gyro: list[tuple[float, float, float]] = []
    all_temp: list[float] = []
    all_ts: list[int] = []

    for record in reader.records:
        if isinstance(record, DataMessage):
            sub = reader.subscriptions.get(record.msg_id)
            if sub is not None and sub.message_name == "imu_sample" and sub.multi_id == 0:
                fmt = reader.formats.get("imu_sample")
                if fmt is not None:
                    decoded = fmt.decode(record.data)
                    all_ts.append(int(decoded["timestamp"]))
                    all_accel.append(
                        (float(decoded["accel"][0]), float(decoded["accel"][1]), float(decoded["accel"][2]))
                    )
                    all_gyro.append(
                        (float(decoded["gyro"][0]), float(decoded["gyro"][1]), float(decoded["gyro"][2]))
                    )
                    all_temp.append(float(decoded["temperature"]))

    if not all_ts:
        raise CheckRefused(f"No imu_sample:0 records found in {log_path}")

    t_first = all_ts[0]
    t_last = all_ts[-1]
    cutoff_start = t_first + 1_000_000
    cutoff_end = t_last - 1_000_000

    kept_indices = [i for i, ts in enumerate(all_ts) if cutoff_start <= ts <= cutoff_end]
    if not kept_indices:
        raise CheckRefused(f"No samples kept after trimming 1 s window in {log_path}")

    first_kept_ts = all_ts[kept_indices[0]]
    last_kept_ts = all_ts[kept_indices[-1]]
    duration_s = (last_kept_ts - first_kept_ts) / 1_000_000.0

    if duration_s < MIN_VALID_S:
        raise CheckRefused(
            f"Under {MIN_VALID_S:.1f} s between first and last sample kept ({duration_s:.2f} s) in {log_path}"
        )

    n = len(kept_indices)
    kept_accel = [all_accel[i] for i in kept_indices]
    kept_gyro = [all_gyro[i] for i in kept_indices]
    kept_temp = [all_temp[i] for i in kept_indices]

    mean_ax = sum(a[0] for a in kept_accel) / n
    mean_ay = sum(a[1] for a in kept_accel) / n
    mean_az = sum(a[2] for a in kept_accel) / n
    mean_accel = (mean_ax, mean_ay, mean_az)

    var_ax = sum((a[0] - mean_ax) ** 2 for a in kept_accel) / n
    var_ay = sum((a[1] - mean_ay) ** 2 for a in kept_accel) / n
    var_az = sum((a[2] - mean_az) ** 2 for a in kept_accel) / n
    accel_std = (math.sqrt(var_ax), math.sqrt(var_ay), math.sqrt(var_az))
    max_accel_std = max(accel_std)

    mean_gx = sum(g[0] for g in kept_gyro) / n
    mean_gy = sum(g[1] for g in kept_gyro) / n
    mean_gz = sum(g[2] for g in kept_gyro) / n
    mean_gyro = (mean_gx, mean_gy, mean_gz)

    var_gx = sum((g[0] - mean_gx) ** 2 for g in kept_gyro) / n
    var_gy = sum((g[1] - mean_gy) ** 2 for g in kept_gyro) / n
    var_gz = sum((g[2] - mean_gz) ** 2 for g in kept_gyro) / n
    gyro_std = (math.sqrt(var_gx), math.sqrt(var_gy), math.sqrt(var_gz))
    max_gyro_std = max(gyro_std)

    mean_temp = sum(kept_temp) / n
    dropouts = len(reader.dropouts)

    if max_accel_std > MAX_ACCEL_STD:
        raise CheckRefused(
            f"Accelerometer std ({max_accel_std:.4f} m/s²) exceeds {MAX_ACCEL_STD:.2f} m/s² in {log_path}: vehicle moved"
        )

    if max_gyro_std > MAX_GYRO_STD:
        raise CheckRefused(
            f"Gyroscope std ({max_gyro_std:.4f} rad/s) exceeds {MAX_GYRO_STD:.2f} rad/s in {log_path}: vehicle moved"
        )

    accel_mag = math.sqrt(mean_ax**2 + mean_ay**2 + mean_az**2)
    if abs(accel_mag - GRAVITY) > MAX_MAGNITUDE_ERROR:
        raise CheckRefused(
            f"Mean accelerometer magnitude ({accel_mag:.3f} m/s²) differs from {GRAVITY:.5f} m/s² "
            f"by more than {MAX_MAGNITUDE_ERROR:.1f} m/s² in {log_path}"
        )

    return PoseSummary(
        name=pose_name,
        rotation=rot_info,
        samples=n,
        dropouts=dropouts,
        mean_accel=mean_accel,
        accel_std=accel_std,
        max_accel_std=max_accel_std,
        mean_gyro=mean_gyro,
        gyro_std=gyro_std,
        max_gyro_std=max_gyro_std,
        mean_temp=mean_temp,
        duration_s=duration_s,
    )


def score_candidates(
    level_summary: PoseSummary,
    nose_summary: PoseSummary,
) -> tuple[CandidateScore, list[CandidateScore]]:
    """Evaluates all 20 rotations against both pose means."""
    if level_summary.rotation.value != nose_summary.rotation.value:
        raise CheckRefused(
            f"SENS_BOARD_ROT differs between logs: {level_summary.rotation.name} ({level_summary.rotation.value}) "
            f"vs {nose_summary.rotation.name} ({nose_summary.rotation.value})"
        )

    dot_poses = (
        level_summary.mean_accel[0] * nose_summary.mean_accel[0]
        + level_summary.mean_accel[1] * nose_summary.mean_accel[1]
        + level_summary.mean_accel[2] * nose_summary.mean_accel[2]
    )
    norm_l = math.sqrt(sum(a**2 for a in level_summary.mean_accel))
    norm_n = math.sqrt(sum(a**2 for a in nose_summary.mean_accel))
    cos_poses = max(-1.0, min(1.0, dot_poses / (norm_l * norm_n)))
    pose_angle_deg = math.degrees(math.acos(cos_poses))
    if pose_angle_deg < MIN_POSE_ANGLE_DEG:
        raise CheckRefused(
            f"Pose angle between level and nose-down is {pose_angle_deg:.1f} deg, "
            f"less than required {MIN_POSE_ANGLE_DEG:.1f} deg"
        )

    R_logged = level_summary.rotation.matrix
    R_logged_inv = mat_transpose(R_logged)

    scores: list[CandidateScore] = []
    for cand in ROTATIONS:
        R_rel = mat_mul_mat(cand.matrix, R_logged_inv)
        v_level = mat_mul_vec(R_rel, level_summary.mean_accel)
        v_nose = mat_mul_vec(R_rel, nose_summary.mean_accel)

        # level_err: angle between v_level and (0, 0, -1)
        norm_v_level = math.sqrt(v_level[0]**2 + v_level[1]**2 + v_level[2]**2)
        dot_level = -v_level[2]
        cos_level = max(-1.0, min(1.0, dot_level / norm_v_level))
        level_err = math.degrees(math.acos(cos_level))

        # nose_err: remove part along v_level direction, take angle with (-1, 0, 0)
        u_level = (v_level[0] / norm_v_level, v_level[1] / norm_v_level, v_level[2] / norm_v_level)
        proj = v_nose[0] * u_level[0] + v_nose[1] * u_level[1] + v_nose[2] * u_level[2]
        v_left = (
            v_nose[0] - proj * u_level[0],
            v_nose[1] - proj * u_level[1],
            v_nose[2] - proj * u_level[2],
        )
        norm_v_left = math.sqrt(v_left[0]**2 + v_left[1]**2 + v_left[2]**2)
        if norm_v_left < 1e-6:
            nose_err = 180.0
        else:
            dot_nose = -v_left[0]
            cos_nose = max(-1.0, min(1.0, dot_nose / norm_v_left))
            nose_err = math.degrees(math.acos(cos_nose))

        fit = max(level_err, nose_err)
        scores.append(CandidateScore(rotation=cand, level_err_deg=level_err, nose_err_deg=nose_err, fit_deg=fit))

    best = min(scores, key=lambda s: s.fit_deg)
    if best.fit_deg > MAX_FIT_DEG:
        raise CheckRefused(
            f"Best rotation's fit ({best.rotation.name} at {best.fit_deg:.1f} deg) exceeds threshold {MAX_FIT_DEG:.1f} deg"
        )

    return best, scores


def format_pose_line(summary: PoseSummary) -> str:
    """Formats one pose summary as a space-separated key=value line."""
    mean_str = f"{summary.mean_accel[0]:.3f},{summary.mean_accel[1]:.3f},{summary.mean_accel[2]:.3f}"
    return (
        f"pose={summary.name} samples={summary.samples} "
        f"dropouts={summary.dropouts} mean={mean_str} "
        f"max_accel_std={summary.max_accel_std:.4f} mean_temp={summary.mean_temp:.2f}"
    )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("level_log", help="path to level pose .ulg log file")
    parser.add_argument("nose_down_log", help="path to nose-down pose .ulg log file")
    parser.add_argument(
        "--serial",
        metavar="PORT",
        default="<port>",
        help="Link serial port for param set command (default: <port>)",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    try:
        args = parse_args(argv)
    except SystemExit as exit_exc:
        return exit_exc.code if exit_exc.code is not None else 2

    try:
        level_summary = read_pose_log(args.level_log, "level")
        nose_summary = read_pose_log(args.nose_down_log, "nose_down")
    except CheckRefused as refused:
        print(f"Refused: {refused}", file=sys.stderr)
        return 1
    except (OSError, ValueError) as io_err:
        print(f"error: {io_err}", file=sys.stderr)
        return 2

    try:
        best, _ = score_candidates(level_summary, nose_summary)
    except CheckRefused as refused:
        print(f"Refused: {refused}", file=sys.stderr)
        return 1

    print(format_pose_line(level_summary))
    print(format_pose_line(nose_summary))
    print(f"logged_rotation={level_summary.rotation.name}")
    print(f"best_rotation={best.rotation.name}")
    print(f"fit={best.fit_deg:.2f} deg")

    fits = (level_summary.rotation.value == best.rotation.value)
    if fits:
        print(f"Logged rotation {level_summary.rotation.name} fits (fit {best.fit_deg:.1f} deg).")
    else:
        target_flag = f"--serial {args.serial}"
        print(
            f"Logged rotation {level_summary.rotation.name} does not fit "
            f"(best is {best.rotation.name} with fit {best.fit_deg:.1f} deg)."
        )
        print(f"tools/link/link_monitor.py {target_flag} param set SENS_BOARD_ROT {best.rotation.name}")
        print("Note: SENS_BOARD_ROT applies only after a reboot.")
        print("Note: changing SENS_BOARD_ROT requires a new magnetometer calibration (see docs/guide/estimation.md).")

    print(
        "Note: assumes the first log was level and the second nose down: "
        "a log with the nose up instead names YAW_180, which the tool cannot tell apart."
    )

    return 0 if fits else 1


if __name__ == "__main__":
    sys.exit(main())
