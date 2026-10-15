#!/usr/bin/env python3
"""
Unit tests for ROS2-UGV Custom Serial Protocol (ADR-0003)

Decoupled into 4 distinct layers:
  1. TestCRC16AndCOBS: Pure bitwise CRC16 and byte stuffing (zero message dependency)
  2. TestFraming: Pure Layer 2 frame packaging, delimiter check, and error rejection (zero message dependency)
  3. TestMessageCodecs: High-level business message serialization and roundtrips
  4. TestStreamDecoder: Continuous serial stream parsing and noise recovery
"""

import struct
import sys
import unittest
from pathlib import Path

# Add protocol directory to python path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ugv_protocol import (
    PROTOCOL_VERSION,
    crc16_ccitt,
    cobs_encode,
    cobs_decode,
    frame_encode,
    frame_decode,
    decode_frame_stream,
    HeadingMode,
    ControlSource,
    OperatingMode,
    BatteryLevel,
    SystemStatus,
    LogLevel,
    MessageId,
    HandshakeReq,
    HandshakeResp,
    Heartbeat,
    HeartbeatAck,
    SetVelocity,
    Telemetry,
    TextLog,
)


class TestCRC16AndCOBS(unittest.TestCase):
    """Layer 1: Pure algorithm verification independent of any framing or messages."""

    def test_crc16_standard_vector(self):
        # Known standard test string "123456789" with CCITT-FALSE (init 0xFFFF, poly 0x1021) -> 0x29B1
        data = b"123456789"
        crc = crc16_ccitt(data, 0xFFFF)
        self.assertEqual(crc, 0x29B1)

    def test_cobs_simple(self):
        data = b"\x01\x02\x00\x03\x04"
        encoded = cobs_encode(data)
        self.assertTrue(encoded.endswith(b"\x00"))
        self.assertNotIn(b"\x00", encoded[:-1])
        decoded = cobs_decode(encoded[:-1])
        self.assertEqual(data, decoded)

    def test_cobs_all_zeros(self):
        data = b"\x00\x00\x00\x00"
        encoded = cobs_encode(data)
        self.assertTrue(encoded.endswith(b"\x00"))
        self.assertNotIn(b"\x00", encoded[:-1])
        decoded = cobs_decode(encoded[:-1])
        self.assertEqual(data, decoded)

    def test_cobs_no_zeros(self):
        data = bytes(range(1, 255))
        encoded = cobs_encode(data)
        self.assertTrue(encoded.endswith(b"\x00"))
        self.assertNotIn(b"\x00", encoded[:-1])
        decoded = cobs_decode(encoded[:-1])
        self.assertEqual(data, decoded)


class TestFraming(unittest.TestCase):
    """Layer 2: Framing encode/decode on raw byte payloads (zero schema dependency)."""

    def test_raw_payload_roundtrip(self):
        raw_payload = b"raw robotics binary payload\x00with\x00zeros"
        frame = frame_encode(raw_payload)
        self.assertTrue(frame.endswith(b"\x00"))
        self.assertNotIn(b"\x00", frame[:-1])
        decoded = frame_decode(frame[:-1])
        self.assertEqual(decoded, raw_payload)

    def test_crc_error_rejection(self):
        raw_payload = b"important control bytes"
        valid_frame = frame_encode(raw_payload)
        # Decode COBS to tamper with payload directly
        raw_with_crc = bytearray(cobs_decode(valid_frame[:-1]))
        raw_with_crc[2] ^= 0xFF  # Corrupt payload byte
        tampered_frame = cobs_encode(bytes(raw_with_crc))

        with self.assertRaises(ValueError):
            frame_decode(tampered_frame[:-1])

    def test_truncated_frame_rejection(self):
        raw_payload = b"payload to be truncated"
        frame = frame_encode(raw_payload)
        # Truncate frame before completion
        truncated = frame[:4]
        with self.assertRaises(Exception):
            frame_decode(truncated)


class TestMessageCodecs(unittest.TestCase):
    """Layer 3: High-level business message serialization and field validation."""

    def test_handshake_roundtrip(self):
        req = HandshakeReq(
            seq=101,
            timestamp_us=500000,
            host_proto_version=PROTOCOL_VERSION,
            expected_heading=HeadingMode.DRIVE_FRONT,
            reserved_padding=0,
        )
        frame = req.encode_frame()
        self.assertTrue(frame.endswith(b"\x00"))
        decoded_req = decode_frame_stream(frame[:-1])
        self.assertEqual(decoded_req.seq, 101)
        self.assertEqual(decoded_req.timestamp_us, 500000)
        self.assertEqual(decoded_req.host_proto_version, PROTOCOL_VERSION)
        self.assertEqual(decoded_req.expected_heading, HeadingMode.DRIVE_FRONT)

    def test_handshake_resp_roundtrip(self):
        uid = (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12)
        resp = HandshakeResp(
            seq=102,
            timestamp_us=500500,
            mcu_proto_version=PROTOCOL_VERSION,
            mcu_heading=HeadingMode.DRIVE_FRONT,
            heading_matched=1,
            hw_version=0x11,
            reserved_padding=0,
            mcu_uid=uid,
        )
        frame = resp.encode_frame()
        decoded_resp = decode_frame_stream(frame[:-1])
        self.assertEqual(decoded_resp.seq, 102)
        self.assertEqual(decoded_resp.mcu_heading, HeadingMode.DRIVE_FRONT)
        self.assertEqual(decoded_resp.heading_matched, 1)
        self.assertEqual(decoded_resp.hw_version, 0x11)
        self.assertEqual(decoded_resp.mcu_uid, uid)

    def test_set_velocity_roundtrip(self):
        cmd = SetVelocity(
            seq=200,
            timestamp_us=1234567,
            target_vx=0.85,
            target_wz=-0.42,
        )
        frame = cmd.encode_frame()
        decoded = decode_frame_stream(frame[:-1])
        self.assertEqual(decoded.seq, 200)
        self.assertAlmostEqual(decoded.target_vx, 0.85, places=5)
        self.assertAlmostEqual(decoded.target_wz, -0.42, places=5)

    def test_telemetry_roundtrip(self):
        telem = Telemetry(
            seq=3000,
            timestamp_us=98765432,
            measured_vx=0.52,
            measured_wz=0.15,
            wheel_vel_l=0.48,
            wheel_vel_r=0.56,
            wheel_pos_l=18500,
            wheel_pos_r=19200,
            imu_accel_x=0.01,
            imu_accel_y=-0.02,
            imu_accel_z=9.81,
            imu_gyro_x=0.001,
            imu_gyro_y=-0.002,
            imu_gyro_z=0.151,
            battery_voltage=24.3,
            battery_level=BatteryLevel.OK,
            control_source=ControlSource.ROS2,
            status_flags=SystemStatus.IMU_READY,
        )
        frame = telem.encode_frame()
        decoded = decode_frame_stream(frame[:-1])
        self.assertEqual(decoded.seq, 3000)
        self.assertEqual(decoded.timestamp_us, 98765432)
        self.assertAlmostEqual(decoded.measured_vx, 0.52, places=5)
        self.assertAlmostEqual(decoded.measured_wz, 0.15, places=5)
        self.assertEqual(decoded.wheel_pos_l, 18500)
        self.assertEqual(decoded.wheel_pos_r, 19200)
        self.assertAlmostEqual(decoded.battery_voltage, 24.3, places=2)
        self.assertEqual(decoded.battery_level, BatteryLevel.OK)
        self.assertEqual(decoded.control_source, ControlSource.ROS2)
        self.assertEqual(decoded.status_flags, SystemStatus.IMU_READY)

    def test_text_log_roundtrip(self):
        msg = "C50C Motor Driver Initialized Successfully."
        log = TextLog(
            seq=401,
            timestamp_us=100000,
            level=LogLevel.INFO,
            length=len(msg),
            reserved=0,
            message=msg,
        )
        frame = log.encode_frame()
        decoded = decode_frame_stream(frame[:-1])
        self.assertEqual(decoded.level, LogLevel.INFO)
        self.assertEqual(decoded.message, msg)


class TestStreamDecoder(unittest.TestCase):
    """Layer 4: Multi-frame stream decoding and noise tolerance."""

    def test_multiple_frames_in_stream(self):
        cmd1 = SetVelocity(seq=1, timestamp_us=100, target_vx=0.5, target_wz=0.0)
        cmd2 = SetVelocity(seq=2, timestamp_us=200, target_vx=1.0, target_wz=-0.5)

        stream = cmd1.encode_frame() + cmd2.encode_frame()
        # Slicing out individual 0x00 delimiters
        packets = [p for p in stream.split(b"\x00") if p]
        self.assertEqual(len(packets), 2)

        decoded1 = decode_frame_stream(packets[0])
        decoded2 = decode_frame_stream(packets[1])
        self.assertEqual(decoded1.seq, 1)
        self.assertEqual(decoded2.seq, 2)


if __name__ == "__main__":
    unittest.main()
