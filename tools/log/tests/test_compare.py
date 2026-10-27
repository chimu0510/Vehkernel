# SPDX-License-Identifier: GPL-3.0-or-later
"""Tests for tools/log/compare.py (spec 0011, ADR-0018 L2)."""

import math
from pathlib import Path
import struct
import sys

import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools" / "log"))

from compare import compare_logs  # noqa: E402
import vkulog  # noqa: E402


def make_attitude_payload(
    ts: int,
    ts_sample: int,
    estimator_id: int = 1,
    q: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0),
    angular_velocity: tuple[float, float, float] = (0.0, 0.0, 0.0),
    gyro_bias: tuple[float, float, float] = (0.0, 0.0, 0.0),
    variance: tuple[float, float, float] = (1e-4, 1e-4, 1e-3),
    accel_rejected: int = 0,
) -> bytes:
    return struct.unpack(
        "<73s",
        struct.pack(
            "<QQB4f3f3f3fI",
            ts,
            ts_sample,
            estimator_id,
            *q,
            *angular_velocity,
            *gyro_bias,
            *variance,
            accel_rejected,
        ),
    )[0]


def make_odometry_payload(
    ts: int,
    ts_sample: int,
    estimator_id: int = 1,
    position: tuple[float, float, float] = (0.0, 0.0, 0.0),
    q: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0),
    velocity: tuple[float, float, float] = (0.0, 0.0, 0.0),
    yaw_rate: float = 0.0,
    position_variance: tuple[float, float, float] = (1e-4, 1e-4, 1e-4),
    heading_variance: float = 1e-3,
    velocity_variance: tuple[float, float, float] = (1e-4, 1e-4, 1e-4),
) -> bytes:
    return struct.unpack(
        "<89s",
        struct.pack(
            "<QQB3f4f3ff3ff3f",
            ts,
            ts_sample,
            estimator_id,
            *position,
            *q,
            *velocity,
            yaw_rate,
            *position_variance,
            heading_variance,
            *velocity_variance,
        ),
    )[0]


def test_exact_pair_passes(tmp_path: Path, capsys):
    """An exact pair of logs passes comparison with exact faithfulness."""
    log_file1 = tmp_path / "log1.ulg"
    log_file2 = tmp_path / "log2.ulg"

    for path in (log_file1, log_file2):
        with vkulog.ULogWriter(path, start_timestamp_us=1_000_000) as w:
            w.write_format(vkulog.FORMATS["estimator_restart"])
            w.write_format(vkulog.FORMATS["vehicle_attitude"])
            w.write_format(vkulog.FORMATS["vehicle_local_odometry"])
            w.write_add_logged(1, "estimator_restart", 0)
            w.write_add_logged(2, "vehicle_attitude", 0)
            w.write_add_logged(3, "vehicle_local_odometry", 0)

            w.write_data(1, struct.pack("<Q", 1_000_000))

            for i in range(5):
                ts = 1_000_000 + i * 20_000
                w.write_data(2, make_attitude_payload(ts, ts))
                w.write_data(3, make_odometry_payload(ts, ts, position=(float(i), 0.0, 0.0)))

    ok = compare_logs(log_file1, log_file2, tolerance=0.0)
    out = capsys.readouterr().out
    assert ok is True
    assert "Faithfulness: exact" in out
    assert "Comparing window: from first estimator_restart at 1000000 us" in out
    assert "Left out of faithfulness check: timestamp" in out


def test_differing_beyond_tolerance_fails_and_names_field(tmp_path: Path, capsys):
    """A pair differing beyond --tolerance fails and names the field."""
    log_file1 = tmp_path / "log1.ulg"
    log_file2 = tmp_path / "log2.ulg"

    with vkulog.ULogWriter(log_file1, start_timestamp_us=1_000_000) as w:
        w.write_format(vkulog.FORMATS["vehicle_attitude"])
        w.write_format(vkulog.FORMATS["vehicle_local_odometry"])
        w.write_add_logged(1, "vehicle_attitude", 0)
        w.write_add_logged(2, "vehicle_local_odometry", 0)
        ts = 1_000_000
        w.write_data(1, make_attitude_payload(ts, ts, gyro_bias=(0.01, 0.0, 0.0)))
        w.write_data(2, make_odometry_payload(ts, ts, position=(1.0, 2.0, 0.0)))

    with vkulog.ULogWriter(log_file2, start_timestamp_us=1_000_000) as w:
        w.write_format(vkulog.FORMATS["vehicle_attitude"])
        w.write_format(vkulog.FORMATS["vehicle_local_odometry"])
        w.write_add_logged(1, "vehicle_attitude", 0)
        w.write_add_logged(2, "vehicle_local_odometry", 0)
        ts = 1_000_000
        # gyro_bias[0] differs by 0.05 (0.06 vs 0.01)
        w.write_data(1, make_attitude_payload(ts, ts, gyro_bias=(0.06, 0.0, 0.0)))
        w.write_data(2, make_odometry_payload(ts, ts, position=(1.0, 2.0, 0.0)))

    ok = compare_logs(log_file1, log_file2, tolerance=0.01)
    out = capsys.readouterr().out
    assert ok is False
    assert "Faithfulness: FAILED" in out
    assert "vehicle_attitude.gyro_bias[0]" in out
    assert "max difference 5.000000e-02 > tolerance 1.000000e-02" in out

    ok2 = compare_logs(log_file1, log_file2, tolerance=0.1)
    out2 = capsys.readouterr().out
    assert ok2 is True
    assert "Faithfulness: within tolerance 0.1" in out2
    assert "vehicle_attitude.gyro_bias[0]: largest difference 5.000000e-02" in out2


def test_nan_both_sides_equal(tmp_path: Path, capsys):
    """NaN on both sides is treated as equal and passes exact comparison."""
    log_file1 = tmp_path / "nan1.ulg"
    log_file2 = tmp_path / "nan2.ulg"

    for path in (log_file1, log_file2):
        with vkulog.ULogWriter(path, start_timestamp_us=1_000_000) as w:
            w.write_format(vkulog.FORMATS["vehicle_attitude"])
            w.write_add_logged(1, "vehicle_attitude", 0)
            ts = 1_000_000
            w.write_data(1, make_attitude_payload(ts, ts, angular_velocity=(float("nan"), 0.0, 0.0)))

    ok = compare_logs(log_file1, log_file2, tolerance=0.0)
    out = capsys.readouterr().out
    assert ok is True
    assert "Faithfulness: exact" in out


def test_nan_one_side_mismatch(tmp_path: Path, capsys):
    """NaN on only one side is a mismatch regardless of tolerance."""
    log_file1 = tmp_path / "nan1.ulg"
    log_file2 = tmp_path / "nan2.ulg"

    with vkulog.ULogWriter(log_file1, start_timestamp_us=1_000_000) as w:
        w.write_format(vkulog.FORMATS["vehicle_attitude"])
        w.write_add_logged(1, "vehicle_attitude", 0)
        ts = 1_000_000
        w.write_data(1, make_attitude_payload(ts, ts, angular_velocity=(float("nan"), 0.0, 0.0)))

    with vkulog.ULogWriter(log_file2, start_timestamp_us=1_000_000) as w:
        w.write_format(vkulog.FORMATS["vehicle_attitude"])
        w.write_add_logged(1, "vehicle_attitude", 0)
        ts = 1_000_000
        w.write_data(1, make_attitude_payload(ts, ts, angular_velocity=(0.0, 0.0, 0.0)))

    ok = compare_logs(log_file1, log_file2, tolerance=100.0)
    out = capsys.readouterr().out
    assert ok is False
    assert "vehicle_attitude.angular_velocity[0]" in out
    assert "NaN mismatch" in out


def test_integer_fields_must_match_regardless_of_tolerance(tmp_path: Path, capsys):
    """Integer and boolean fields must match exactly even if tolerance is large."""
    log_file1 = tmp_path / "int1.ulg"
    log_file2 = tmp_path / "int2.ulg"

    with vkulog.ULogWriter(log_file1, start_timestamp_us=1_000_000) as w:
        w.write_format(vkulog.FORMATS["vehicle_attitude"])
        w.write_add_logged(1, "vehicle_attitude", 0)
        ts = 1_000_000
        w.write_data(1, make_attitude_payload(ts, ts, estimator_id=1, accel_rejected=0))

    with vkulog.ULogWriter(log_file2, start_timestamp_us=1_000_000) as w:
        w.write_format(vkulog.FORMATS["vehicle_attitude"])
        w.write_add_logged(1, "vehicle_attitude", 0)
        ts = 1_000_000
        # estimator_id differs: 2 vs 1
        w.write_data(1, make_attitude_payload(ts, ts, estimator_id=2, accel_rejected=1))

    ok = compare_logs(log_file1, log_file2, tolerance=10.0)
    out = capsys.readouterr().out
    assert ok is False
    assert "vehicle_attitude.estimator_id" in out
    assert "integer/bool mismatch" in out


def test_empty_comparison_fails(tmp_path: Path, capsys):
    """A comparison matching no samples in the window fails with an informative message."""
    log_file1 = tmp_path / "empty1.ulg"
    log_file2 = tmp_path / "empty2.ulg"

    with vkulog.ULogWriter(log_file1, start_timestamp_us=1_000_000) as w:
        w.write_format(vkulog.FORMATS["estimator_restart"])
        w.write_add_logged(1, "estimator_restart", 0)
        w.write_data(1, struct.pack("<Q", 1_000_000))

    with vkulog.ULogWriter(log_file2, start_timestamp_us=1_000_000) as w:
        w.write_format(vkulog.FORMATS["estimator_restart"])
        w.write_add_logged(1, "estimator_restart", 0)
        w.write_data(1, struct.pack("<Q", 1_000_000))

    ok = compare_logs(log_file1, log_file2, tolerance=0.0)
    out = capsys.readouterr().out
    assert ok is False
    assert "Faithfulness: FAILED (no matching samples in compared window)" in out


def test_metrics_match_hand_computed_values_with_nonzero_start_pose(tmp_path: Path, capsys):
    """Metrics correctly align truth with nonzero initial position and yaw at restart."""
    truth_file = tmp_path / "truth.ulg"
    replayed_file = tmp_path / "replayed.ulg"

    # Truth start at t=1_000_000:
    # Position: (50.0, 100.0, 0.0)
    # Heading: facing east (yaw = pi / 2) -> q = (cos(pi/4), 0, 0, sin(pi/4))
    t0 = 1_000_000
    q_east = (math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4))

    # Truth end at t=1_100_000:
    # Vehicle drove forward (east) by 5.0 m -> world pos (50.0, 105.0, 0.0)
    # Vehicle turned right by 90 deg -> facing south (yaw = pi) -> q = (0, 0, 0, 1)
    t1 = 1_100_000
    q_south = (0.0, 0.0, 0.0, 1.0)

    with vkulog.ULogWriter(truth_file, start_timestamp_us=t0) as w:
        w.write_format(vkulog.FORMATS["vehicle_attitude_groundtruth"])
        w.write_format(vkulog.FORMATS["vehicle_local_odometry_groundtruth"])
        w.write_add_logged(1, "vehicle_attitude_groundtruth", 0)
        w.write_add_logged(2, "vehicle_local_odometry_groundtruth", 0)

        # Truth at t0
        w.write_data(1, struct.pack("<Q4f3f", t0, *q_east, 0.0, 0.0, 0.0))
        w.write_data(2, struct.pack("<Q3f4f3f3f", t0, 50.0, 100.0, 0.0, *q_east, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

        # Truth at t1
        w.write_data(1, struct.pack("<Q4f3f", t1, *q_south, 0.0, 0.0, 0.0))
        w.write_data(2, struct.pack("<Q3f4f3f3f", t1, 50.0, 105.0, 0.0, *q_south, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

    # In estimate's frame:
    # Origin is at t0: pos (0, 0, 0), yaw = 0
    # At t1:
    # Aligned truth position: driven 5.0 m along X -> (5.0, 0.0, 0.0)
    # Aligned truth yaw: turned 90 deg right -> yaw = pi / 2
    # Aligned truth attitude quaternion: yaw = pi / 2 -> q_east_aligned
    #
    # Replayed odometry at t1 has error:
    # Position: (5.03, 0.04, 0.0) -> error = hypot(0.03, 0.04) = 0.05 m!
    # Heading: yaw = pi/2 + 3 deg -> heading error = 3.0 deg!
    rad_odom_yaw = math.pi / 2 + math.radians(3.0)
    q_rep_odom = (math.cos(rad_odom_yaw / 2), 0.0, 0.0, math.sin(rad_odom_yaw / 2))

    # Replayed attitude at t1 has error:
    # Attitude yaw: pi/2 + 5 deg -> attitude error = 5.0 deg!
    rad_att_yaw = math.pi / 2 + math.radians(5.0)
    q_rep_att = (math.cos(rad_att_yaw / 2), 0.0, 0.0, math.sin(rad_att_yaw / 2))

    with vkulog.ULogWriter(replayed_file, start_timestamp_us=t0) as w:
        w.write_format(vkulog.FORMATS["vehicle_attitude"])
        w.write_format(vkulog.FORMATS["vehicle_local_odometry"])
        w.write_add_logged(1, "vehicle_attitude", 0)
        w.write_add_logged(2, "vehicle_local_odometry", 0)

        # Sample at t0
        w.write_data(1, make_attitude_payload(t0, t0, q=(1.0, 0.0, 0.0, 0.0)))
        w.write_data(2, make_odometry_payload(t0, t0, position=(0.0, 0.0, 0.0)))

        # Sample at t1
        w.write_data(1, make_attitude_payload(t1, t1, q=q_rep_att))
        w.write_data(2, make_odometry_payload(t1, t1, position=(5.03, 0.04, 0.0), q=q_rep_odom))

    compare_logs(replayed_file, replayed_file, tolerance=0.0, truth_path=truth_file)
    out = capsys.readouterr().out
    assert "Metrics: attitude error mean=2.5000 deg, max=5.0000 deg, end=5.0000 deg" in out
    assert "Metrics: position error at end=0.0500 m, heading error at end=3.0000 deg" in out
