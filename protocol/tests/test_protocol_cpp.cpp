/**
 * @file test_protocol_cpp.cpp
 * @brief Unit tests for C++ PacketParser, streaming, and C++ API (Layer 4).
 */

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "ugv_protocol.hpp"

using namespace ugv::protocol;

static void test_cpp_encode_and_parser()
{
    printf("[TEST] C++ encode() and PacketParser single packet...\n");
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
    telem.status_flags = static_cast<uint16_t>(SystemStatus::IMU_READY);

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
        assert(t->status_flags == static_cast<uint16_t>(SystemStatus::IMU_READY));
        received = true;
    });

    assert(received);
    printf("  Single packet parser [PASS]\n");
}

static void test_cpp_byte_by_byte_streaming()
{
    printf("[TEST] C++ PacketParser byte-by-byte streaming...\n");
    SetVelocity cmd{};
    cmd.header.seq = 77;
    cmd.target_vx = 1.0f;
    cmd.target_wz = -0.5f;

    std::vector<uint8_t> encoded = encode(cmd);

    PacketParser parser;
    size_t packet_count = 0;
    for (uint8_t b : encoded) {
        parser.parse_byte(b, [&](MessageId id, const uint8_t* payload, size_t len) {
            assert(id == MessageId::SetVelocity);
            assert(len == sizeof(SetVelocity));
            const auto* c = reinterpret_cast<const SetVelocity*>(payload);
            assert(c->header.seq == 77);
            assert(std::fabs(c->target_vx - 1.0f) < 1e-5f);
            packet_count++;
        });
    }

    assert(packet_count == 1);
    printf("  Byte-by-byte parsing [PASS]\n");
}

static void test_cpp_back_to_back_packets()
{
    printf("[TEST] C++ PacketParser back-to-back packets in single chunk...\n");
    HandshakeReq req{};
    req.header.seq = 1;
    req.host_proto_version = PROTOCOL_VERSION;

    SetVelocity cmd{};
    cmd.header.seq = 2;
    cmd.target_vx = 0.5f;

    Heartbeat hb{};
    hb.header.seq = 3;

    std::vector<uint8_t> stream;
    auto f1 = encode(req);
    auto f2 = encode(cmd);
    auto f3 = encode(hb);

    stream.insert(stream.end(), f1.begin(), f1.end());
    stream.insert(stream.end(), f2.begin(), f2.end());
    stream.insert(stream.end(), f3.begin(), f3.end());

    PacketParser parser;
    std::vector<MessageId> received_ids;
    parser.parse_chunk(stream.data(), stream.size(), [&](MessageId id, const uint8_t*, size_t) {
        received_ids.push_back(id);
    });

    assert(received_ids.size() == 3);
    assert(received_ids[0] == MessageId::HandshakeReq);
    assert(received_ids[1] == MessageId::SetVelocity);
    assert(received_ids[2] == MessageId::Heartbeat);
    printf("  3 consecutive packets parsed cleanly [PASS]\n");
}

static void test_cpp_noise_recovery()
{
    printf("[TEST] C++ PacketParser noise recovery...\n");
    SetVelocity cmd{};
    cmd.header.seq = 101;
    cmd.target_vx = 0.8f;
    std::vector<uint8_t> valid_frame = encode(cmd);

    // Stream with garbage bytes, then a zero delimiter (representing dropped/corrupt byte sequence), then a valid frame
    std::vector<uint8_t> noisy_stream = {0x12, 0x34, 0x56, 0x78, 0x00};
    noisy_stream.insert(noisy_stream.end(), valid_frame.begin(), valid_frame.end());

    PacketParser parser;
    size_t valid_count = 0;
    parser.parse_chunk(noisy_stream.data(), noisy_stream.size(), [&](MessageId id, const uint8_t* payload, size_t) {
        if (id == MessageId::SetVelocity) {
            const auto* c = reinterpret_cast<const SetVelocity*>(payload);
            assert(c->header.seq == 101);
            valid_count++;
        }
    });

    assert(valid_count == 1);
    printf("  Resynchronization after noise [PASS]\n");
}

int main()
{
    printf("========================================\n");
    printf(" Running C++ Stream Parser Tests (Layer 4)\n");
    printf("========================================\n");
    test_cpp_encode_and_parser();
    test_cpp_byte_by_byte_streaming();
    test_cpp_back_to_back_packets();
    test_cpp_noise_recovery();
    printf("==> ALL C++ PROTOCOL TESTS PASSED!\n\n");
    return 0;
}
