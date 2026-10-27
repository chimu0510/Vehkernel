# SPDX-License-Identifier: GPL-3.0-or-later
"""Tests for the vkulog ULog version 1 reader and writer (spec 0011, ADR-0019)."""

import struct
from pathlib import Path

import pytest
import pyulog

import vkulog


def test_round_trip_every_message_kind(tmp_path: Path):
    """Write a log with every message kind (I, F, A, P, D, O, S) through vkulog,
    read it back unchanged through vkulog, and read with pyulog."""
    log_file = tmp_path / "round_trip.ulg"
    start_time = 1_000_000

    # 1. Write the log with every message kind through vkulog
    with vkulog.ULogWriter(log_file, start_timestamp_us=start_time) as w:
        # F: Formats from schema
        w.write_format(vkulog.FORMATS["imu_sample"])
        w.write_format(vkulog.FORMATS["body_velocity_measurement"])

        # I: Information
        w.write_info("sys_name", "c50c-v1_1")
        w.write_info("hw_version", 1)
        w.write_info("rate_scale", 1.0)

        # P: Parameters
        w.write_parameter("ATT_KP", 0.5)
        w.write_parameter("COM_ARM_CHECK", 1)

        # A: Add logged messages
        w.write_add_logged(msg_id=1, message_name="imu_sample", multi_id=0)
        w.write_add_logged(msg_id=2, message_name="body_velocity_measurement", multi_id=0)

        # D: Data message 1
        # imu_sample: uint64 timestamp, uint8 device_instance, float[3] accel, float[3] gyro, float temperature
        imu_data = struct.pack("<QB3f3ff", 1_000_100, 0, 0.0, 0.0, 9.81, 0.01, -0.02, 0.03, 28.5)
        w.write_data(msg_id=1, data=imu_data)

        # Parameter change after data message
        w.write_parameter("ATT_KP", 0.75)

        # O: Dropout
        w.write_dropout(duration_ms=25)

        # S: Sync
        w.write_sync()

        # D: Data message 2
        # body_velocity_measurement: uint64 timestamp, uint64 timestamp_sample, uint8 source,
        # float vx, float vy, float yaw_rate, float[3] variance, float residual
        vel_data = struct.pack("<QQBfffffff", 1_000_200, 1_000_150, 0, 1.2, -0.1, 0.05, 1e-4, 1e-4, 1e-3, 0.0)
        w.write_data(msg_id=2, data=vel_data)

    # 2. Read back unchanged through vkulog
    reader = vkulog.ULogReader(log_file)
    assert reader.version == 1
    assert reader.start_timestamp_us == start_time
    assert reader.flag_bits is not None

    # Formats (F)
    assert "imu_sample" in reader.formats
    assert "body_velocity_measurement" in reader.formats
    assert reader.formats["imu_sample"].fields[0].name == "timestamp"
    assert reader.formats["imu_sample"].fields[2].name == "accel"
    assert reader.formats["imu_sample"].fields[2].array_size == 3

    # Information (I)
    assert reader.info["sys_name"] == "c50c-v1_1"
    assert reader.info["hw_version"] == 1
    assert abs(reader.info["rate_scale"] - 1.0) < 1e-5

    # Parameters (P)
    assert abs(reader.parameters["ATT_KP"] - 0.5) < 1e-5
    assert reader.parameters["COM_ARM_CHECK"] == 1
    assert len(reader.parameter_changes) == 1
    assert reader.parameter_changes[0] == (1_000_100, "ATT_KP", 0.75)
    assert reader.parameter_changes[0].name == "ATT_KP"
    assert abs(reader.parameter_changes[0].value - 0.75) < 1e-5
    assert reader.parameter_changes[0].timestamp_us == 1_000_100

    # Add logged (A)
    assert 1 in reader.subscriptions
    assert reader.subscriptions[1].message_name == "imu_sample"
    assert reader.subscriptions[1].multi_id == 0
    assert 2 in reader.subscriptions
    assert reader.subscriptions[2].message_name == "body_velocity_measurement"

    # Data (D)
    assert len(reader.data_messages) == 2
    assert reader.data_messages[0].msg_id == 1
    assert reader.data_messages[0].timestamp_us == 1_000_100
    assert reader.data_messages[0].data == imu_data
    assert reader.data_messages[1].msg_id == 2
    assert reader.data_messages[1].timestamp_us == 1_000_200
    assert reader.data_messages[1].data == vel_data

    # Dropout (O)
    assert len(reader.dropouts) == 1
    assert reader.dropouts[0].duration_ms == 25

    # Sync (S)
    assert reader.sync_count == 1

    # 3. pyulog reads the same file with its formats, parameters, information and dropouts
    u = pyulog.ULog(str(log_file))
    assert "imu_sample" in u.message_formats
    assert "body_velocity_measurement" in u.message_formats
    assert u.msg_info_dict["sys_name"] == "c50c-v1_1"
    assert abs(u.initial_parameters["ATT_KP"] - 0.5) < 1e-5
    assert u.initial_parameters["COM_ARM_CHECK"] == 1
    assert len(u.changed_parameters) == 1
    assert u.changed_parameters[0] == (1_000_100, "ATT_KP", 0.75)
    assert len(u.dropouts) == 1
    assert u.dropouts[0].duration == 25
    assert len(u.data_list) == 2
    # Verify data parsed by pyulog matches
    dataset_imu = u.get_dataset("imu_sample")
    assert dataset_imu.data["timestamp"][0] == 1_000_100
    assert abs(dataset_imu.data["accel[2]"][0] - 9.81) < 1e-4
    assert abs(dataset_imu.data["temperature"][0] - 28.5) < 1e-4

    dataset_vel = u.get_dataset("body_velocity_measurement")
    assert dataset_vel.data["timestamp"][0] == 1_000_200
    assert abs(dataset_vel.data["vx"][0] - 1.2) < 1e-4


def test_corrupt_files():
    """Verify reader error handling on malformed files."""
    with pytest.raises(ValueError, match="too short"):
        vkulog.ULogReader(b"ULog")

    with pytest.raises(ValueError, match="Invalid ULog magic"):
        vkulog.ULogReader(b"BadMagicBytes00000000000000")
