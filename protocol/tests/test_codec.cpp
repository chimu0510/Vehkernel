#include <cassert>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <vector>

#include "ugv_protocol/frame.h"
#include "ugv_protocol.hpp"

extern "C" {
#include "ugv_protocol.h"
}

using namespace ugv::protocol;

static void test_c_struct_sizes() {
    printf("[TEST] Checking C struct sizes...\n");
    assert(sizeof(ugv_msg_header_t) == 10);
    assert(sizeof(ugv_msg_handshake_req_t) == 10 + 4);
    assert(sizeof(ugv_msg_handshake_resp_t) == 10 + 18);
    assert(sizeof(ugv_msg_heartbeat_t) == 10 + 4);
    assert(sizeof(ugv_msg_heartbeat_ack_t) == 10 + 8);
    assert(sizeof(ugv_msg_set_velocity_t) == 10 + 8);
    assert(sizeof(ugv_msg_telemetry_t) == 10 + 56);
    printf("  Header: %zu bytes\n", sizeof(ugv_msg_header_t));
    printf("  HandshakeReq: %zu bytes\n", sizeof(ugv_msg_handshake_req_t));
    printf("  HandshakeResp: %zu bytes\n", sizeof(ugv_msg_handshake_resp_t));
    printf("  SetVelocity: %zu bytes\n", sizeof(ugv_msg_set_velocity_t));
    printf("  Telemetry: %zu bytes\n", sizeof(ugv_msg_telemetry_t));
    printf("  Max payload: %zu bytes\n", UGV_MAX_PAYLOAD_SIZE);
    printf("  Max frame: %zu bytes\n", UGV_MAX_FRAME_SIZE);
}

static void test_c_roundtrip() {
    printf("[TEST] Testing C encode/decode roundtrip...\n");
    ugv_msg_set_velocity_t cmd = {};
    cmd.header.msg_id = UGV_MSG_SET_VELOCITY;
    cmd.header.seq = 105;
    cmd.header.timestamp_us = 456789;
    cmd.target_vx = 1.25f;
    cmd.target_wz = -0.75f;

    uint8_t frame_buf[UGV_MAX_FRAME_SIZE] = {};
    size_t frame_len = ugv_encode_frame(&cmd, sizeof(cmd), frame_buf);
    assert(frame_len > 0);
    assert(frame_buf[frame_len - 1] == 0x00);

    ugv_msg_payload_t decoded = {};
    int payload_len = ugv_decode_frame(frame_buf, frame_len - 1, &decoded);
    assert(payload_len == (int)sizeof(cmd));
    assert(decoded.header.msg_id == UGV_MSG_SET_VELOCITY);
    assert(decoded.header.seq == 105);
    assert(decoded.header.timestamp_us == 456789);
    assert(std::fabs(decoded.set_velocity.target_vx - 1.25f) < 1e-5f);
    assert(std::fabs(decoded.set_velocity.target_wz - (-0.75f)) < 1e-5f);
    printf("  C roundtrip PASS!\n");
}

static void test_cpp_packet_parser() {
    printf("[TEST] Testing C++ PacketParser streaming...\n");
    Telemetry telem{};
    telem.header.seq = 999;
    telem.header.timestamp_us = 12345678;
    telem.measured_vx = 0.65f;
    telem.measured_wz = 0.32f;
    telem.wheel_pos_l = 10000;
    telem.wheel_pos_r = 10200;
    telem.battery_voltage = 24.2f;
    telem.battery_level = static_cast<uint8_t>(BatteryLevel::OK);
    telem.control_source = static_cast<uint8_t>(ControlSource::ROS2);
    telem.status_flags = static_cast<uint16_t>(SystemStatusFlags::FLAG_IMU_READY);

    std::vector<uint8_t> encoded = encode(telem);
    assert(!encoded.empty());
    assert(encoded.back() == 0x00);

    PacketParser parser;
    bool received = false;
    parser.parse_chunk(encoded.data(), encoded.size(), [&](MessageId id, const uint8_t* payload, size_t len) {
        assert(id == MessageId::Telemetry);
        assert(len == sizeof(Telemetry));
        const auto* t = reinterpret_cast<const Telemetry*>(payload);
        assert(t->header.seq == 999);
        assert(t->header.timestamp_us == 12345678);
        assert(std::fabs(t->measured_vx - 0.65f) < 1e-5f);
        assert(std::fabs(t->measured_wz - 0.32f) < 1e-5f);
        assert(t->wheel_pos_l == 10000);
        assert(t->wheel_pos_r == 10200);
        assert(t->battery_level == static_cast<uint8_t>(BatteryLevel::OK));
        received = true;
    });

    assert(received);
    printf("  C++ PacketParser PASS!\n");
}

int main() {
    printf("=== Running UGV Protocol C/C++ Codec Verification ===\n");
    test_c_struct_sizes();
    test_c_roundtrip();
    test_cpp_packet_parser();
    printf("=== ALL C/C++ PROTOCOL TESTS PASSED! ===\n");
    return 0;
}
