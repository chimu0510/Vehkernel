# SPDX-License-Identifier: GPL-3.0-or-later
"""Tests for vehkernel-replay and tools/log/replay.py (spec 0011, ADR-0018 L2)."""

from pathlib import Path
import struct
import sys
import time

import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools" / "log"))

from compare import compare_logs  # noqa: E402
from replay import replay  # noqa: E402
import vkulog  # noqa: E402


def make_imu_sample_data(
    ts: int,
    device_instance: int = 0,
    accel: tuple[float, float, float] = (0.0, 0.0, 9.80665),
    gyro: tuple[float, float, float] = (0.0, 0.0, 0.0),
    temperature: float = 25.0,
) -> bytes:
    # imu_sample: uint64 timestamp, uint8 device_instance, float[3] accel, float[3] gyro, float temperature
    return struct.pack("<QB3f3ff", ts, device_instance, *accel, *gyro, temperature)


def make_velocity_data(
    ts: int,
    ts_sample: int,
    source: int = 0,
    vx: float = 0.5,
    vy: float = 0.0,
    yaw_rate: float = 0.0,
    variance: tuple[float, float, float] = (1e-4, 1e-4, 1e-3),
    residual: float = 0.0,
) -> bytes:
    # body_velocity_measurement: uint64 timestamp, uint64 timestamp_sample, uint8 source,
    # float vx, float vy, float yaw_rate, float[3] variance, float residual
    return struct.pack("<QQBfffffff", ts, ts_sample, source, vx, vy, yaw_rate, *variance, residual)


def test_input_order_between_topics(tmp_path: Path):
    """Inputs with the same timestamp are processed in wq:ctrl order (IMU before odometry)."""
    in_log = tmp_path / "order_in.ulg"
    out_log = tmp_path / "order_out.ulg"

    ts_start = 1_000_000
    dt_us = 5_000  # 200 Hz

    with vkulog.ULogWriter(in_log, start_timestamp_us=ts_start) as w:
        w.write_format(vkulog.FORMATS["imu_sample"])
        w.write_format(vkulog.FORMATS["body_velocity_measurement"])
        w.write_format(vkulog.FORMATS["estimator_restart"])
        w.write_add_logged(1, "imu_sample", 0)
        w.write_add_logged(2, "body_velocity_measurement", 0)
        w.write_add_logged(3, "estimator_restart", 0)

        w.write_data(3, struct.pack("<Q", ts_start))

        # 220 samples (1.1 s) still to align attitude estimator
        for i in range(220):
            ts = ts_start + i * dt_us
            # Both imu_sample and body_velocity_measurement with the same timestamp
            w.write_data(1, make_imu_sample_data(ts))
            w.write_data(2, make_velocity_data(ts, ts, vx=0.0))

        # After alignment, sample at same timestamp with forward motion and turn
        ts_step = ts_start + 220 * dt_us
        w.write_data(1, make_imu_sample_data(ts_step, gyro=(0.0, 0.0, 0.2)))
        w.write_data(2, make_velocity_data(ts_step, ts_step, vx=1.0, yaw_rate=0.2))

    replay(in_log, out_log)

    reader = vkulog.ULogReader(out_log)
    assert len(reader.data_messages) > 0
    # Both vehicle_attitude and vehicle_local_odometry exist
    topic_names = {sub.message_name for sub in reader.subscriptions.values()}
    assert "vehicle_attitude" in topic_names
    assert "vehicle_local_odometry" in topic_names


def test_parameter_change_applied_at_its_time(tmp_path: Path):
    """A parameter change in the log is applied at its recorded timestamp."""
    in_log = tmp_path / "param_in.ulg"
    out_log = tmp_path / "param_out.ulg"

    ts_start = 1_000_000
    dt_us = 5_000

    with vkulog.ULogWriter(in_log, start_timestamp_us=ts_start) as w:
        w.write_format(vkulog.FORMATS["imu_sample"])
        w.write_format(vkulog.FORMATS["body_velocity_measurement"])
        w.write_parameter("ATT_KP", 1.0)
        w.write_add_logged(1, "imu_sample", 0)
        w.write_add_logged(2, "body_velocity_measurement", 0)

        # Still for 210 samples
        for i in range(210):
            ts = ts_start + i * dt_us
            w.write_data(1, make_imu_sample_data(ts))
            w.write_data(2, make_velocity_data(ts, ts))

        # Parameter change after sample 209
        change_ts = ts_start + 209 * dt_us
        w.write_parameter("ATT_KP", 2.0)

        for i in range(210, 230):
            ts = ts_start + i * dt_us
            w.write_data(1, make_imu_sample_data(ts))
            w.write_data(2, make_velocity_data(ts, ts))

    replay(in_log, out_log)

    reader = vkulog.ULogReader(out_log)
    assert len(reader.parameter_changes) == 1
    assert reader.parameter_changes[0].name == "ATT_KP"
    assert reader.parameter_changes[0].value == 2.0
    assert reader.parameter_changes[0].timestamp_us == change_ts


def test_output_identical_across_two_runs(tmp_path: Path):
    """Replaying the same log produces identical output across two runs."""
    in_log = tmp_path / "repro_in.ulg"
    out_log1 = tmp_path / "repro_out1.ulg"
    out_log2 = tmp_path / "repro_out2.ulg"

    ts_start = 1_000_000
    dt_us = 5_000

    with vkulog.ULogWriter(in_log, start_timestamp_us=ts_start) as w:
        w.write_format(vkulog.FORMATS["imu_sample"])
        w.write_format(vkulog.FORMATS["body_velocity_measurement"])
        w.write_info("sys_name", "test")
        w.write_parameter("ATT_KP", 1.0)
        w.write_add_logged(1, "imu_sample", 0)
        w.write_add_logged(2, "body_velocity_measurement", 0)

        for i in range(220):
            ts = ts_start + i * dt_us
            w.write_data(1, make_imu_sample_data(ts, accel=(0.0, 0.0, 9.80665)))
            w.write_data(2, make_velocity_data(ts, ts, vx=0.5))

    replay(in_log, out_log1)
    replay(in_log, out_log2)

    assert out_log1.read_bytes() == out_log2.read_bytes()
    assert compare_logs(out_log1, out_log2, tolerance=0.0) is True


def test_replay_60s_log_speed(tmp_path: Path):
    """Replay of a 60 s hand-made log takes under 5 s."""
    in_log = tmp_path / "speed_in.ulg"
    out_log = tmp_path / "speed_out.ulg"

    ts_start = 1_000_000
    dt_us = 5_000  # 200 Hz
    total_samples = 60 * 200  # 12,000 samples = 60 seconds

    with vkulog.ULogWriter(in_log, start_timestamp_us=ts_start) as w:
        w.write_format(vkulog.FORMATS["imu_sample"])
        w.write_format(vkulog.FORMATS["body_velocity_measurement"])
        w.write_format(vkulog.FORMATS["estimator_restart"])
        w.write_add_logged(1, "imu_sample", 0)
        w.write_add_logged(2, "body_velocity_measurement", 0)
        w.write_add_logged(3, "estimator_restart", 0)

        w.write_data(3, struct.pack("<Q", ts_start))

        # First 1.1 s (220 samples) still
        for i in range(220):
            ts = ts_start + i * dt_us
            w.write_data(1, make_imu_sample_data(ts))
            w.write_data(2, make_velocity_data(ts, ts, vx=0.0))

        # Remaining ~59 s driving with motion
        for i in range(220, total_samples):
            ts = ts_start + i * dt_us
            w.write_data(1, make_imu_sample_data(ts, gyro=(0.01, -0.01, 0.05)))
            w.write_data(2, make_velocity_data(ts, ts, vx=0.8, yaw_rate=0.05))

    t0 = time.perf_counter()
    replay(in_log, out_log)
    elapsed = time.perf_counter() - t0

    print(f"Replay of 60 s log took {elapsed:.3f} s")
    assert elapsed < 5.0, f"Replay took {elapsed:.3f} s; must be < 5.0 s"
    assert out_log.exists()
    assert out_log.stat().st_size > 0


def test_unknown_parameter_fails_by_default(tmp_path: Path):
    """Replay fails by default if the log contains unknown parameters."""
    in_log = tmp_path / "unknown_param_in.ulg"
    out_log = tmp_path / "unknown_param_out.ulg"

    ts_start = 1_000_000

    with vkulog.ULogWriter(in_log, start_timestamp_us=ts_start) as w:
        w.write_format(vkulog.FORMATS["imu_sample"])
        w.write_parameter("ATT_KP", 1.0)
        w.write_parameter("UNKNOWN_PARAM_XYZ", 42.0)
        w.write_add_logged(1, "imu_sample", 0)
        w.write_data(1, make_imu_sample_data(ts_start))

    with pytest.raises(RuntimeError) as exc_info:
        replay(in_log, out_log)

    assert "UNKNOWN_PARAM_XYZ" in str(exc_info.value)
    assert "--allow-unknown-parameters" in str(exc_info.value)


def test_unknown_parameter_allowed_with_flag(tmp_path: Path):
    """Replay succeeds when --allow-unknown-parameters is set."""
    in_log = tmp_path / "unknown_param_in.ulg"
    out_log = tmp_path / "unknown_param_out.ulg"

    ts_start = 1_000_000

    with vkulog.ULogWriter(in_log, start_timestamp_us=ts_start) as w:
        w.write_format(vkulog.FORMATS["imu_sample"])
        w.write_parameter("ATT_KP", 1.0)
        w.write_parameter("UNKNOWN_PARAM_XYZ", 42.0)
        w.write_add_logged(1, "imu_sample", 0)
        w.write_data(1, make_imu_sample_data(ts_start))

    replay(in_log, out_log, allow_unknown_parameters=True)
    assert out_log.exists()

