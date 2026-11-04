# SPDX-License-Identifier: GPL-3.0-or-later
"""Tests for tools/log/accel_check.py (#188)."""

from __future__ import annotations

import math
from pathlib import Path
import re
import struct
import sys

import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools" / "log"))
sys.path.insert(0, str(REPO / "tools" / "link"))

from accel_check import (  # noqa: E402
    GRAVITY,
    ROTATIONS,
    ROTATIONS_BY_NAME,
    RotationInfo,
    main,
    mat_mul_mat,
    mat_mul_vec,
    mat_transpose,
)
import vkulog  # noqa: E402


def make_imu_payload(
    timestamp_us: int,
    device_instance: int = 0,
    accel: tuple[float, float, float] = (0.0, 0.0, -9.80665),
    gyro: tuple[float, float, float] = (0.0, 0.0, 0.0),
    temperature: float = 25.0,
) -> bytes:
    """Packs imu_sample binary payload."""
    return struct.pack("<QB3f3ff", timestamp_us, device_instance, *accel, *gyro, temperature)


def write_test_log(
    path: Path,
    samples: list[tuple[int, tuple[float, float, float], tuple[float, float, float], float]],
    board_rot: int | str | None = 0,
    parameter_changes: list[tuple[str, int | float]] | None = None,
    dropouts: list[int] | None = None,
    start_timestamp_us: int = 1_000_000,
) -> None:
    """Helper to write a synthetic ULog file containing imu_sample:0 and parameters."""
    with vkulog.ULogWriter(path, start_timestamp_us=start_timestamp_us) as w:
        w.write_format(vkulog.FORMATS["imu_sample"])

        if board_rot is not None:
            if isinstance(board_rot, str):
                val = ROTATIONS_BY_NAME[board_rot].value
            else:
                val = int(board_rot)
            w.write_parameter("SENS_BOARD_ROT", val)

        w.write_add_logged(1, "imu_sample", 0)

        for ts, accel, gyro, temp in samples:
            w.write_data(1, make_imu_payload(ts, 0, accel, gyro, temp))

        if dropouts:
            for d in dropouts:
                w.write_dropout(d)

        if parameter_changes:
            for name, val in parameter_changes:
                w.write_parameter(name, val)


def make_samples(
    duration_s: float = 8.0,
    rate_hz: float = 100.0,
    accel_mean: tuple[float, float, float] = (0.0, 0.0, -GRAVITY),
    accel_square_amp: float = 0.005,
    gyro_mean: tuple[float, float, float] = (0.0, 0.0, 0.0),
    gyro_square_amp: float = 0.002,
    temperature: float = 25.0,
    start_ts_us: int = 1_000_000,
) -> list[tuple[int, tuple[float, float, float], tuple[float, float, float], float]]:
    """Generates synthetic IMU samples with exact mean and deterministic alternating std."""
    n = int(round(duration_s * rate_hz))
    dt_us = int(round(1_000_000 / rate_hz))

    samples = []
    for i in range(n):
        ts = start_ts_us + i * dt_us
        sign = 1.0 if (i % 2 == 0) else -1.0
        ax = accel_mean[0] + sign * accel_square_amp
        ay = accel_mean[1] + sign * accel_square_amp
        az = accel_mean[2] + sign * accel_square_amp

        gx = gyro_mean[0] + sign * gyro_square_amp
        gy = gyro_mean[1] + sign * gyro_square_amp
        gz = gyro_mean[2] + sign * gyro_square_amp

        samples.append((ts, (ax, ay, az), (gx, gy, gz), temperature))
    return samples


def make_pose_pair_logs(
    tmp_path: Path,
    prefix: str,
    true_rot: RotationInfo,
    logged_rot: RotationInfo,
    pitch_deg: float = 45.0,
    accel_offset: tuple[float, float, float] = (0.0, 0.0, 0.0),
    duration_s: float = 8.0,
    accel_std: float = 0.005,
    gyro_std: float = 0.002,
    gyro_bias: tuple[float, float, float] = (0.0, 0.0, 0.0),
    level_mag_scale: float = 1.0,
    nose_mag_scale: float = 1.0,
    dropouts: list[int] | None = None,
) -> tuple[Path, Path]:
    """Generates a pair of synthetic logs (level and nose-down) for true_rot and logged_rot."""
    # Body-frame specific force
    body_level = (
        (0.0 + accel_offset[0]) * level_mag_scale,
        (0.0 + accel_offset[1]) * level_mag_scale,
        (-GRAVITY + accel_offset[2]) * level_mag_scale,
    )

    pitch_rad = math.radians(pitch_deg)
    body_nose = (
        (-GRAVITY * math.sin(pitch_rad) + accel_offset[0]) * nose_mag_scale,
        (0.0 + accel_offset[1]) * nose_mag_scale,
        (-GRAVITY * math.cos(pitch_rad) + accel_offset[2]) * nose_mag_scale,
    )

    # Transform body to device through true_rot: a_device = R_true^T * a_body
    R_true_inv = mat_transpose(true_rot.matrix)
    dev_level = mat_mul_vec(R_true_inv, body_level)
    dev_nose = mat_mul_vec(R_true_inv, body_nose)

    # Firmware applies logged_rot to device readings: a_logged = R_logged * a_device
    logged_level_mean = mat_mul_vec(logged_rot.matrix, dev_level)
    logged_nose_mean = mat_mul_vec(logged_rot.matrix, dev_nose)

    level_samples = make_samples(
        duration_s=duration_s,
        accel_mean=logged_level_mean,
        accel_square_amp=accel_std,
        gyro_mean=gyro_bias,
        gyro_square_amp=gyro_std,
    )
    nose_samples = make_samples(
        duration_s=duration_s,
        accel_mean=logged_nose_mean,
        accel_square_amp=accel_std,
        gyro_mean=gyro_bias,
        gyro_square_amp=gyro_std,
    )

    level_path = tmp_path / f"{prefix}_level.ulg"
    nose_path = tmp_path / f"{prefix}_nose.ulg"

    write_test_log(level_path, level_samples, board_rot=logged_rot.value, dropouts=dropouts)
    write_test_log(nose_path, nose_samples, board_rot=logged_rot.value, dropouts=dropouts)

    return level_path, nose_path


def test_rotations_table_against_source():
    """Validates the 20 rotations table against drivers/params.toml and hal/include/vk/hal/rotation.hpp."""
    # 1. Parse drivers/params.toml for names and values
    toml_path = REPO / "drivers" / "params.toml"
    if not toml_path.is_file():
        pytest.skip("firmware sources not present: these tests run from a host tools bundle")
    text = toml_path.read_text(encoding="utf-8")
    matches = re.findall(r'\{\s*name\s*=\s*"([^"]+)",\s*value\s*=\s*(\d+)\s*\}', text)
    assert len(matches) == 20
    for name, val_str in matches:
        val = int(val_str)
        assert name in ROTATIONS_BY_NAME, f"Missing {name} in ROTATIONS"
        assert ROTATIONS_BY_NAME[name].value == val, f"Value mismatch for {name}"

    # 2. Parse hal/include/vk/hal/rotation.hpp and evaluate expressions
    hpp_path = REPO / "hal" / "include" / "vk" / "hal" / "rotation.hpp"
    hpp_text = hpp_path.read_text(encoding="utf-8")

    case_matches = re.findall(r"case\s+Rotation::(\w+):\s*return\s*\{([^}]+)\};", hpp_text)
    assert len(case_matches) == 20, f"Expected 20 cases, found {len(case_matches)}"

    by_stripped = {r.name.replace("_", "").lower(): r for r in ROTATIONS}

    unit_vectors = [
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
    ]

    for name_pascal, exprs_raw in case_matches:
        key = name_pascal.lower()
        assert key in by_stripped, f"Unknown case {name_pascal} in rotation.hpp"
        rot = by_stripped[key]

        expr_list = [e.strip() for e in exprs_raw.split(",")]
        assert len(expr_list) == 3, f"Expected 3 expressions for {name_pascal}, got {expr_list}"

        for col, (ux, uy, uz) in enumerate(unit_vectors):
            env = {
                "__builtins__": {},
                "h": math.sqrt(0.5),
                "x": ux,
                "y": uy,
                "z": uz,
            }
            evaluated_col = [float(eval(expr, env)) for expr in expr_list]
            for row in range(3):
                expected = rot.matrix[row][col]
                diff = abs(evaluated_col[row] - expected)
                assert diff < 1e-6, (
                    f"Mismatch in {rot.name} ({name_pascal}) row {row}, col {col}: "
                    f"eval={evaluated_col[row]}, matrix={expected}"
                )


def test_rotations_table_is_20_distinct_proper_rotations():
    """All 20 matrices are orthonormal with determinant +1 and distinct."""

    def det3(m: tuple[tuple[float, float, float], ...]) -> float:
        return (
            m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
        )

    assert len(ROTATIONS) == 20
    matrices = [r.matrix for r in ROTATIONS]

    for r in ROTATIONS:
        m = r.matrix
        mt = mat_transpose(m)
        prod = mat_mul_mat(m, mt)
        for row in range(3):
            for col in range(3):
                expected = 1.0 if row == col else 0.0
                assert abs(prod[row][col] - expected) < 1e-5, f"{r.name} is not orthonormal"
        d = det3(m)
        assert abs(d - 1.0) < 1e-5, f"{r.name} determinant {d} != +1.0"

    for i in range(len(matrices)):
        for j in range(i + 1, len(matrices)):
            diff = sum(abs(matrices[i][r][c] - matrices[j][r][c]) for r in range(3) for c in range(3))
            assert diff > 0.1, f"Rotations {ROTATIONS[i].name} and {ROTATIONS[j].name} are duplicate"


def test_all_20_rotations_matching_logged_exit_0(tmp_path: Path, capsys):
    """Each of the 20 rotations as the logged value on a matching log: exit 0."""
    for rot in ROTATIONS:
        level_p, nose_p = make_pose_pair_logs(tmp_path, f"match_{rot.name}", rot, rot)
        code = main([str(level_p), str(nose_p)])
        captured = capsys.readouterr()
        assert code == 0, f"Expected 0 for {rot.name}, got {code}: {captured.err}"
        assert f"Logged rotation {rot.name} fits" in captured.out
        assert f"logged_rotation={rot.name}" in captured.out
        assert f"best_rotation={rot.name}" in captured.out


def test_all_20_rotations_identified_when_different_logged(tmp_path: Path, capsys):
    """Each of the 20 as the true one, with another logged: exit 1 and the right name printed."""
    for true_rot in ROTATIONS:
        other_rot = ROTATIONS_BY_NAME["YAW_90"] if true_rot.name == "NONE" else ROTATIONS_BY_NAME["NONE"]
        level_p, nose_p = make_pose_pair_logs(tmp_path, f"diff_{true_rot.name}", true_rot, other_rot)
        code = main([str(level_p), str(nose_p)])
        captured = capsys.readouterr()
        assert code == 1, f"Expected 1 for true {true_rot.name} with logged {other_rot.name}, got {code}"
        assert f"Logged rotation {other_rot.name} does not fit" in captured.out
        assert f"best is {true_rot.name}" in captured.out
        assert f"best_rotation={true_rot.name}" in captured.out
        assert f"param set SENS_BOARD_ROT {true_rot.name}" in captured.out


def test_c50c_offsets_tilt_35_and_90(tmp_path: Path, capsys):
    """A nose tilt of 35 deg and of 90 deg, with the C50C's offsets (-0.105, 0.019, -0.304) m/s^2: still fits."""
    c50c_offset = (-0.105, 0.019, -0.304)
    rot = ROTATIONS_BY_NAME["NONE"]

    # 1. 35 deg tilt: the offsets shrink the measured angle by about a degree, so 30 deg itself is refused.
    l35, n35 = make_pose_pair_logs(
        tmp_path, "c50c_35", rot, rot, pitch_deg=35.0, accel_offset=c50c_offset
    )
    code35 = main([str(l35), str(n35)])
    cap35 = capsys.readouterr()
    assert code35 == 0, f"Tilt 35 with C50C offset failed: {cap35.err}"
    assert "fits" in cap35.out

    # 2. 90 deg tilt
    l90, n90 = make_pose_pair_logs(
        tmp_path, "c50c_90", rot, rot, pitch_deg=90.0, accel_offset=c50c_offset
    )
    code90 = main([str(l90), str(n90)])
    cap90 = capsys.readouterr()
    assert code90 == 0, f"Tilt 90 with C50C offset failed: {cap90.err}"
    assert "fits" in cap90.out


def test_refuse_moved_accelerometer(tmp_path: Path, capsys):
    """Accelerometer std above 0.15 m/s^2 refused, below accepted."""
    rot = ROTATIONS_BY_NAME["NONE"]

    # Side A: std 0.14 m/s^2 (< 0.15) -> accepted
    l_ok, n_ok = make_pose_pair_logs(tmp_path, "accel_ok", rot, rot, accel_std=0.14)
    assert main([str(l_ok), str(n_ok)]) == 0

    # Side B: std 0.16 m/s^2 (> 0.15) -> refused
    l_bad, n_bad = make_pose_pair_logs(tmp_path, "accel_bad", rot, rot, accel_std=0.16)
    assert main([str(l_bad), str(n_bad)]) == 1
    captured = capsys.readouterr()
    assert "Accelerometer std" in captured.err
    assert "vehicle moved" in captured.err


def test_refuse_moved_gyroscope(tmp_path: Path, capsys):
    """Gyroscope std above 0.02 rad/s refused, below accepted."""
    rot = ROTATIONS_BY_NAME["NONE"]

    # Side A: std 0.018 rad/s (< 0.02) -> accepted
    l_ok, n_ok = make_pose_pair_logs(tmp_path, "gyro_ok", rot, rot, gyro_std=0.018)
    assert main([str(l_ok), str(n_ok)]) == 0

    # Side B: std 0.022 rad/s (> 0.02) -> refused
    l_bad, n_bad = make_pose_pair_logs(tmp_path, "gyro_bad", rot, rot, gyro_std=0.022)
    assert main([str(l_bad), str(n_bad)]) == 1
    captured = capsys.readouterr()
    assert "Gyroscope std" in captured.err
    assert "vehicle moved" in captured.err


def test_refuse_magnitude_error(tmp_path: Path, capsys):
    """Mean magnitude differing from 9.80665 by > 1.0 m/s^2 refused, <= 1.0 accepted."""
    rot = ROTATIONS_BY_NAME["NONE"]

    # Side A: error 0.9 m/s^2 (< 1.0) -> accepted
    scale_ok = (GRAVITY + 0.9) / GRAVITY
    l_ok, n_ok = make_pose_pair_logs(tmp_path, "mag_ok", rot, rot, level_mag_scale=scale_ok)
    assert main([str(l_ok), str(n_ok)]) == 0

    # Side B1: high error 1.1 m/s^2 (> 1.0) -> refused
    scale_high = (GRAVITY + 1.1) / GRAVITY
    l_high, n_high = make_pose_pair_logs(tmp_path, "mag_high", rot, rot, level_mag_scale=scale_high)
    assert main([str(l_high), str(n_high)]) == 1
    captured = capsys.readouterr()
    assert "Mean accelerometer magnitude" in captured.err

    # Side B2: low error 1.1 m/s^2 (> 1.0) -> refused
    scale_low = (GRAVITY - 1.1) / GRAVITY
    l_low, n_low = make_pose_pair_logs(tmp_path, "mag_low", rot, rot, level_mag_scale=scale_low)
    assert main([str(l_low), str(n_low)]) == 1
    captured = capsys.readouterr()
    assert "Mean accelerometer magnitude" in captured.err


def test_refuse_pose_angle(tmp_path: Path, capsys):
    """Two means under 30 deg apart refused, >= 30 deg accepted."""
    rot = ROTATIONS_BY_NAME["NONE"]

    # Side A: 28 deg (< 30) -> refused
    l_small, n_small = make_pose_pair_logs(tmp_path, "angle_28", rot, rot, pitch_deg=28.0)
    assert main([str(l_small), str(n_small)]) == 1
    captured = capsys.readouterr()
    assert "Pose angle between level and nose-down" in captured.err
    assert "less than required 30.0 deg" in captured.err

    # Side B: 32 deg (> 30) -> accepted
    l_big, n_big = make_pose_pair_logs(tmp_path, "angle_32", rot, rot, pitch_deg=32.0)
    assert main([str(l_big), str(n_big)]) == 0


def test_refuse_too_little_data(tmp_path: Path, capsys):
    """Under 5.0 s between first and last sample kept refused, >= 5.0 s accepted."""
    rot = ROTATIONS_BY_NAME["NONE"]

    # 1 s trimmed on each side.
    # Total duration 6.5 s -> kept duration 4.49 s (< 5.0 s) -> refused
    l_short, n_short = make_pose_pair_logs(tmp_path, "short", rot, rot, duration_s=6.5)
    assert main([str(l_short), str(n_short)]) == 1
    captured = capsys.readouterr()
    assert "Under 5.0 s between first and last sample kept" in captured.err

    # Total duration 7.5 s -> kept duration 5.49 s (> 5.0 s) -> accepted
    l_long, n_long = make_pose_pair_logs(tmp_path, "long", rot, rot, duration_s=7.5)
    assert main([str(l_long), str(n_long)]) == 0


def test_refuse_differing_rotation(tmp_path: Path, capsys):
    """Differing SENS_BOARD_ROT between logs is refused."""
    rot_a = ROTATIONS_BY_NAME["NONE"]
    rot_b = ROTATIONS_BY_NAME["YAW_90"]

    l_a, _ = make_pose_pair_logs(tmp_path, "rot_a", rot_a, rot_a)
    _, n_b = make_pose_pair_logs(tmp_path, "rot_b", rot_b, rot_b)

    assert main([str(l_a), str(n_b)]) == 1
    captured = capsys.readouterr()
    assert "SENS_BOARD_ROT differs between logs" in captured.err


def test_refuse_parameter_change(tmp_path: Path, capsys):
    """A parameter change to SENS_BOARD_ROT during the log is refused."""
    rot = ROTATIONS_BY_NAME["NONE"]
    samples = make_samples(duration_s=8.0)

    log_file = tmp_path / "param_change.ulg"
    write_test_log(
        log_file,
        samples,
        board_rot=rot.value,
        parameter_changes=[("SENS_BOARD_ROT", 2)],
    )

    good_level, _ = make_pose_pair_logs(tmp_path, "good_param", rot, rot)
    assert main([str(good_level), str(log_file)]) == 1
    captured = capsys.readouterr()
    assert "Parameter SENS_BOARD_ROT changed during" in captured.err


def test_refuse_missing_parameter(tmp_path: Path, capsys):
    """Missing SENS_BOARD_ROT parameter is refused."""
    rot = ROTATIONS_BY_NAME["NONE"]
    samples = make_samples(duration_s=8.0)

    log_missing = tmp_path / "missing_param.ulg"
    write_test_log(log_missing, samples, board_rot=None)

    good_level, _ = make_pose_pair_logs(tmp_path, "good_missing", rot, rot)
    assert main([str(good_level), str(log_missing)]) == 1
    captured = capsys.readouterr()
    assert "Missing SENS_BOARD_ROT parameter" in captured.err


def test_gyroscope_bias_does_not_refuse(tmp_path: Path, capsys):
    """A gyroscope bias of 0.03 rad/s with a small std does not refuse."""
    rot = ROTATIONS_BY_NAME["NONE"]
    bias = (0.03, -0.03, 0.03)  # bias well above MAX_GYRO_STD (0.02)
    l_bias, n_bias = make_pose_pair_logs(
        tmp_path, "gyro_bias", rot, rot, gyro_bias=bias, gyro_std=0.005
    )

    code = main([str(l_bias), str(n_bias)])
    captured = capsys.readouterr()
    assert code == 0, f"Expected 0 with gyro bias, got {code}: {captured.err}"
    assert "fits" in captured.out


def test_swapped_logs_refused(tmp_path: Path, capsys):
    """The two logs swapped: refused, since no proper rotation fits."""
    rot = ROTATIONS_BY_NAME["NONE"]
    level_log, nose_log = make_pose_pair_logs(tmp_path, "swapped", rot, rot, pitch_deg=45.0)

    code = main([str(nose_log), str(level_log)])
    captured = capsys.readouterr()
    assert code == 1, f"Expected 1 for swapped logs, got {code}"
    assert "Refused:" in captured.err
    assert "exceeds threshold 15.0 deg" in captured.err


def test_dropouts_reported_and_not_refused(tmp_path: Path, capsys):
    """A log with dropouts: reported and not refused."""
    rot = ROTATIONS_BY_NAME["NONE"]
    level_log, nose_log = make_pose_pair_logs(
        tmp_path, "dropouts", rot, rot, dropouts=[50, 100, 150]
    )

    code = main([str(level_log), str(nose_log)])
    captured = capsys.readouterr()
    assert code == 0, f"Expected 0 with dropouts, got {code}: {captured.err}"
    assert "dropouts=3" in captured.out
    assert "fits" in captured.out


def test_cli_io_errors(tmp_path: Path, capsys):
    """CLI exit code 2 on missing files."""
    code = main(["non_existent_1.ulg", "non_existent_2.ulg"])
    captured = capsys.readouterr()
    assert code == 2
    assert "error:" in captured.err
