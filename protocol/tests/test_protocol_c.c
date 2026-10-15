/**
 * @file test_protocol_c.c
 * @brief Unit tests for C protocol structs and serialization (Layer 3).
 */

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ugv_protocol.h"

static void test_struct_sizes(void)
{
    printf("[TEST] Checking C message struct sizes...\n");
    assert(sizeof(ugv_msg_header_t) == 10);
    assert(sizeof(ugv_msg_handshake_req_t) == 10 + 4);
    assert(sizeof(ugv_msg_handshake_resp_t) == 10 + 18);
    assert(sizeof(ugv_msg_heartbeat_t) == 10 + 4);
    assert(sizeof(ugv_msg_heartbeat_ack_t) == 10 + 8);
    assert(sizeof(ugv_msg_set_velocity_t) == 10 + 8);
    assert(sizeof(ugv_msg_telemetry_t) == 10 + 56);
    assert(sizeof(ugv_msg_set_pid_config_t) == 10 + 32);
    assert(sizeof(ugv_msg_set_heading_config_t) == 10 + 4);
    assert(sizeof(ugv_msg_text_log_t) == 10 + 68);

    assert(UGV_MAX_PAYLOAD_SIZE >= sizeof(ugv_msg_text_log_t));
    assert(UGV_MAX_FRAME_SIZE >= FRAME_MAX_ENCODED(UGV_MAX_PAYLOAD_SIZE));
    printf("  All struct sizes verified successfully [PASS]\n");
}

static void test_handshake_roundtrip(void)
{
    printf("[TEST] Handshake request/response roundtrip...\n");
    ugv_msg_handshake_req_t req = {
        .header = {
            .msg_id = UGV_MSG_HANDSHAKE_REQ,
            .reserved = 0,
            .seq = 1,
            .timestamp_us = 1000,
        },
        .host_proto_version = UGV_PROTOCOL_VERSION,
        .expected_heading = (uint8_t)UGV_HEADING_MODE_DRIVE_FRONT,
        .reserved_padding = 0,
    };

    uint8_t frame[UGV_MAX_FRAME_SIZE];
    size_t flen = ugv_encode_frame(&req, sizeof(req), frame);
    assert(flen > 0);
    assert(frame[flen - 1] == 0x00);

    ugv_msg_payload_t decoded;
    int plen = ugv_decode_frame(frame, flen - 1, &decoded);
    assert(plen == (int)sizeof(req));
    assert(decoded.header.msg_id == UGV_MSG_HANDSHAKE_REQ);
    assert(decoded.header.seq == 1);
    assert(decoded.handshake_req.host_proto_version == UGV_PROTOCOL_VERSION);
    assert(decoded.handshake_req.expected_heading == UGV_HEADING_MODE_DRIVE_FRONT);
    printf("  HandshakeReq roundtrip [PASS]\n");
}

static void test_set_velocity_roundtrip(void)
{
    printf("[TEST] SetVelocity message roundtrip...\n");
    ugv_msg_set_velocity_t cmd = {
        .header = {
            .msg_id = UGV_MSG_SET_VELOCITY,
            .reserved = 0,
            .seq = 42,
            .timestamp_us = 998877,
        },
        .target_vx = 0.75f,
        .target_wz = -1.20f,
    };

    uint8_t frame[UGV_MAX_FRAME_SIZE];
    size_t flen = ugv_encode_frame(&cmd, sizeof(cmd), frame);
    assert(flen > 0);

    ugv_msg_payload_t decoded;
    int plen = ugv_decode_frame(frame, flen - 1, &decoded);
    assert(plen == (int)sizeof(cmd));
    assert(decoded.header.msg_id == UGV_MSG_SET_VELOCITY);
    assert(decoded.header.seq == 42);
    assert(decoded.header.timestamp_us == 998877);
    assert(fabsf(decoded.set_velocity.target_vx - 0.75f) < 1e-5f);
    assert(fabsf(decoded.set_velocity.target_wz - (-1.20f)) < 1e-5f);
    printf("  SetVelocity roundtrip [PASS]\n");
}

static void test_telemetry_roundtrip(void)
{
    printf("[TEST] Telemetry message roundtrip...\n");
    ugv_msg_telemetry_t telem = {
        .header = {
            .msg_id = UGV_MSG_TELEMETRY,
            .reserved = 0,
            .seq = 1000,
            .timestamp_us = 1234567,
        },
        .measured_vx = 0.50f,
        .measured_wz = 0.12f,
        .wheel_vel_l = 0.48f,
        .wheel_vel_r = 0.52f,
        .wheel_pos_l = 36000,
        .wheel_pos_r = -36000,
        .imu_accel_x = 0.01f,
        .imu_accel_y = -0.02f,
        .imu_accel_z = 9.80f,
        .imu_gyro_x = 0.001f,
        .imu_gyro_y = -0.001f,
        .imu_gyro_z = 0.120f,
        .battery_voltage = 24.5f,
        .battery_level = (uint8_t)UGV_BATTERY_LEVEL_OK,
        .control_source = (uint8_t)UGV_CONTROL_SOURCE_ROS2,
        .status_flags = (uint16_t)UGV_SYSTEM_STATUS_IMU_READY,
    };

    uint8_t frame[UGV_MAX_FRAME_SIZE];
    size_t flen = ugv_encode_frame(&telem, sizeof(telem), frame);
    assert(flen > 0);

    ugv_msg_payload_t decoded;
    int plen = ugv_decode_frame(frame, flen - 1, &decoded);
    assert(plen == (int)sizeof(telem));
    assert(decoded.header.msg_id == UGV_MSG_TELEMETRY);
    assert(decoded.header.seq == 1000);
    assert(fabsf(decoded.telemetry.battery_voltage - 24.5f) < 1e-3f);
    assert(decoded.telemetry.wheel_pos_l == 36000);
    assert(decoded.telemetry.wheel_pos_r == -36000);
    assert(decoded.telemetry.status_flags == UGV_SYSTEM_STATUS_IMU_READY);
    printf("  Telemetry roundtrip [PASS]\n");
}

int main(void)
{
    printf("========================================\n");
    printf(" Running C Message Codec Tests (Layer 3)\n");
    printf("========================================\n");
    test_struct_sizes();
    test_handshake_roundtrip();
    test_set_velocity_roundtrip();
    test_telemetry_roundtrip();
    printf("==> ALL C PROTOCOL TESTS PASSED!\n\n");
    return 0;
}
