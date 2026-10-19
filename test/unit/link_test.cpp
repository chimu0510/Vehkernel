// Unit tests for the custom protocol: COBS, CRC16, FrameReader/Writer, and Session (ADR-0002).
#include <vk/link/cobs.hpp>
#include <vk/link/crc16.hpp>
#include <vk/link/frame.hpp>
#include <vk/link/session.hpp>
#include <vk/msg/arming_state.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace {

using namespace vk::link;

// --- COBS tests ---

TEST(Cobs, EncodesAndDecodesKnownVectors)
{
    struct Vector {
        std::vector<std::uint8_t> unencoded;
        std::vector<std::uint8_t> encoded;
    };

    const std::vector<Vector> vectors = {
        // Empty input encodes to single block length byte 0x01.
        {{}, {0x01}},
        // Single zero byte.
        {{0x00}, {0x01, 0x01}},
        // Two zero bytes.
        {{0x00, 0x00}, {0x01, 0x01, 0x01}},
        // Zero, non-zero, zero.
        {{0x00, 0x11, 0x00}, {0x01, 0x02, 0x11, 0x01}},
        // Interleaved non-zero and zero.
        {{0x11, 0x22, 0x00, 0x33}, {0x03, 0x11, 0x22, 0x02, 0x33}},
        // No zeros.
        {{0x11, 0x22, 0x33, 0x44}, {0x05, 0x11, 0x22, 0x33, 0x44}},
    };

    for (const auto& v : vectors) {
        std::vector<std::uint8_t> out(cobs_max_encoded_size(v.unencoded.size()));
        const std::size_t enc_len = cobs_encode(v.unencoded, out);
        ASSERT_EQ(enc_len, v.encoded.size());
        EXPECT_EQ(std::vector<std::uint8_t>(out.begin(), out.begin() + enc_len), v.encoded);

        std::vector<std::uint8_t> dec(v.unencoded.size() + 10);
        const std::size_t dec_len = cobs_decode(v.encoded, dec);
        ASSERT_EQ(dec_len, v.unencoded.size());
        EXPECT_EQ(std::vector<std::uint8_t>(dec.begin(), dec.begin() + dec_len), v.unencoded);
    }
}

TEST(Cobs, LongBlockWithoutZeros)
{
    // 254 non-zero bytes fill exactly one maximal COBS block.
    std::vector<std::uint8_t> in254(254, 0xAA);
    std::vector<std::uint8_t> out254(cobs_max_encoded_size(in254.size()));
    const std::size_t enc_len254 = cobs_encode(in254, out254);
    ASSERT_EQ(enc_len254, 255U);
    EXPECT_EQ(out254[0], 0xFF);

    std::vector<std::uint8_t> dec254(254);
    const std::size_t dec_len254 = cobs_decode(std::span(out254).first(enc_len254), dec254);
    EXPECT_EQ(dec_len254, 254U);
    EXPECT_EQ(dec254, in254);

    // 255 non-zero bytes overflow into a second block.
    std::vector<std::uint8_t> in255(255, 0xBB);
    std::vector<std::uint8_t> out255(cobs_max_encoded_size(in255.size()));
    const std::size_t enc_len255 = cobs_encode(in255, out255);
    ASSERT_EQ(enc_len255, 257U);
    EXPECT_EQ(out255[0], 0xFF);
    EXPECT_EQ(out255[255], 0x02);
    EXPECT_EQ(out255[256], 0xBB);

    std::vector<std::uint8_t> dec255(255);
    const std::size_t dec_len255 = cobs_decode(std::span(out255).first(enc_len255), dec255);
    EXPECT_EQ(dec_len255, 255U);
    EXPECT_EQ(dec255, in255);
}

TEST(Cobs, RejectsOutputBufferTooSmall)
{
    const std::array<std::uint8_t, 3> in = {1, 2, 3};
    std::array<std::uint8_t, 2> small_out{};
    EXPECT_EQ(cobs_encode(in, small_out), 0U);

    const std::array<std::uint8_t, 4> enc = {4, 1, 2, 3};
    std::array<std::uint8_t, 2> small_dec{};
    EXPECT_EQ(cobs_decode(enc, small_dec), 0U);
}

TEST(Cobs, RejectsCorruptedEncoding)
{
    std::array<std::uint8_t, 16> dec{};
    // Code 0 is invalid in COBS stream.
    const std::array<std::uint8_t, 2> zero_code = {0, 1};
    EXPECT_EQ(cobs_decode(zero_code, dec), 0U);

    // Zero byte inside payload block.
    const std::array<std::uint8_t, 3> zero_in_block = {3, 0, 1};
    EXPECT_EQ(cobs_decode(zero_in_block, dec), 0U);

    // Code byte pointing past the end of input.
    const std::array<std::uint8_t, 2> overshoot = {5, 1};
    EXPECT_EQ(cobs_decode(overshoot, dec), 0U);
}

// --- CRC16 tests ---

TEST(Crc16, MatchesKnownVector)
{
    // Standard test vector "123456789" for CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF).
    const std::string_view standard_vector = "123456789";
    const auto span = std::span<const std::uint8_t>(
        reinterpret_cast<const std::uint8_t*>(standard_vector.data()), standard_vector.size());
    EXPECT_EQ(crc16(span), 0x29B1U);
}

TEST(Crc16, HandlesEmptyInput)
{
    EXPECT_EQ(crc16({}), 0xFFFFU);
}

// --- Frame Writer and Reader tests ---

TEST(Frame, WriterProducesWireFrameWithDelimiter)
{
    FrameWriter writer;
    const std::array<std::uint8_t, 4> body = {0xAA, 0xBB, 0xCC, 0xDD};
    std::array<std::uint8_t, max_wire_size> wire{};

    const std::size_t size = writer.write(FrameType::Hello, body, wire);
    ASSERT_GT(size, 0U);
    EXPECT_EQ(wire[size - 1], 0);  // Delimiter byte
    EXPECT_EQ(writer.next_sequence(), 1U);
}

TEST(Frame, WriterRejectsOversizedBodyOrTooSmallBuffer)
{
    FrameWriter writer;
    const std::vector<std::uint8_t> big_body(max_body_size + 1, 0x11);
    std::array<std::uint8_t, max_wire_size> wire{};
    EXPECT_EQ(writer.write(FrameType::Hello, big_body, wire), 0U);

    const std::array<std::uint8_t, 4> normal_body = {1, 2, 3, 4};
    std::array<std::uint8_t, 4> tiny_wire{};
    EXPECT_EQ(writer.write(FrameType::Hello, normal_body, tiny_wire), 0U);
}

TEST(Frame, ReaderParsesValidFrame)
{
    FrameWriter writer;
    const std::array<std::uint8_t, 3> body = {0x10, 0x20, 0x30};
    std::array<std::uint8_t, max_wire_size> wire{};
    const std::size_t size = writer.write(FrameType::TopicData, body, wire);
    ASSERT_GT(size, 0U);

    FrameReader reader;
    std::vector<Frame> received;
    reader.feed(std::span(wire).first(size), [&](const Frame& f) { received.push_back(f); });

    ASSERT_EQ(received.size(), 1U);
    EXPECT_EQ(received[0].version, protocol_version);
    EXPECT_EQ(received[0].type, FrameType::TopicData);
    EXPECT_EQ(received[0].sequence, 0U);
    EXPECT_EQ(received[0].body.size(), 3U);
    EXPECT_EQ(received[0].body[0], 0x10);
    EXPECT_EQ(reader.stats().frames, 1U);
    EXPECT_EQ(reader.stats().crc_errors, 0U);
    EXPECT_EQ(reader.stats().malformed, 0U);
    EXPECT_EQ(reader.stats().lost, 0U);
}

TEST(Frame, ReaderHandlesConcatenatedFrames)
{
    FrameWriter writer;
    std::vector<std::uint8_t> stream;
    for (int i = 0; i < 3; ++i) {
        std::array<std::uint8_t, max_wire_size> wire{};
        const std::array<std::uint8_t, 1> body = {static_cast<std::uint8_t>(i)};
        const std::size_t size = writer.write(FrameType::TopicData, body, wire);
        stream.insert(stream.end(), wire.begin(), wire.begin() + size);
    }

    FrameReader reader;
    std::vector<std::uint16_t> seqs;
    reader.feed(stream, [&](const Frame& f) { seqs.push_back(f.sequence); });

    EXPECT_EQ(seqs, (std::vector<std::uint16_t>{0, 1, 2}));
    EXPECT_EQ(reader.stats().frames, 3U);
}

TEST(Frame, ReaderIgnoresConsecutiveDelimiters)
{
    FrameReader reader;
    std::size_t count = 0;
    const std::array<std::uint8_t, 4> delimiters = {0, 0, 0, 0};
    reader.feed(delimiters, [&](const Frame&) { ++count; });
    EXPECT_EQ(count, 0U);
    EXPECT_EQ(reader.stats().frames, 0U);
    EXPECT_EQ(reader.stats().malformed, 0U);
}

TEST(Frame, ReaderCatchesCrcError)
{
    FrameWriter writer;
    const std::array<std::uint8_t, 3> body = {1, 2, 3};
    std::array<std::uint8_t, max_wire_size> wire{};
    const std::size_t size = writer.write(FrameType::TopicData, body, wire);

    // Corrupt one byte before the delimiter.
    wire[size / 2] ^= 0xFF;

    FrameReader reader;
    std::size_t count = 0;
    reader.feed(std::span(wire).first(size), [&](const Frame&) { ++count; });

    EXPECT_EQ(count, 0U);
    EXPECT_EQ(reader.stats().crc_errors, 1U);
}

TEST(Frame, ReaderCatchesMalformedInput)
{
    FrameReader reader;
    std::size_t count = 0;

    // Truncated frame (too short for header + CRC).
    const std::array<std::uint8_t, 3> truncated = {0x02, 0x01, 0x00};
    reader.feed(truncated, [&](const Frame&) { ++count; });
    EXPECT_EQ(count, 0U);
    EXPECT_EQ(reader.stats().malformed, 1U);
}

TEST(Frame, ReaderCatchesOversizedInput)
{
    FrameReader reader;
    std::size_t count = 0;

    // Stream longer than max_wire_size without a zero delimiter.
    std::vector<std::uint8_t> too_long(max_wire_size + 10, 0x01);
    too_long.push_back(0);  // Close the frame
    reader.feed(too_long, [&](const Frame&) { ++count; });

    EXPECT_EQ(count, 0U);
    EXPECT_EQ(reader.stats().malformed, 1U);
}

TEST(Frame, ReaderRecoversAtNextDelimiterAfterCorruption)
{
    FrameWriter writer;
    std::array<std::uint8_t, max_wire_size> valid_wire{};
    const std::array<std::uint8_t, 2> body = {0x42, 0x43};
    const std::size_t valid_size = writer.write(FrameType::Hello, body, valid_wire);

    // Corrupted bytes followed by zero delimiter, then the valid frame.
    std::vector<std::uint8_t> stream = {0xFF, 0xFE, 0xFD, 0x00};
    stream.insert(stream.end(), valid_wire.begin(), valid_wire.begin() + valid_size);

    FrameReader reader;
    std::vector<FrameType> received;
    reader.feed(stream, [&](const Frame& f) { received.push_back(f.type); });

    ASSERT_EQ(received.size(), 1U);
    EXPECT_EQ(received[0], FrameType::Hello);
    EXPECT_EQ(reader.stats().frames, 1U);
    EXPECT_EQ(reader.stats().malformed, 1U);
}

TEST(Frame, ReaderCountsLostFramesAndHandlesPeerRestart)
{
    FrameReader reader;
    std::size_t count = 0;

    auto send_seq = [&](std::uint16_t seq) {
        std::array<std::uint8_t, max_frame_size> plain{};
        BodyWriter bw(plain);
        bw.u8(protocol_version);
        bw.u8(static_cast<std::uint8_t>(FrameType::TopicData));
        bw.u16(seq);
        bw.u16(crc16(bw.written()));

        std::array<std::uint8_t, max_wire_size> wire{};
        const std::size_t enc_size = cobs_encode(bw.written(), wire);
        wire[enc_size] = 0;
        reader.feed(std::span(wire).first(enc_size + 1), [&](const Frame&) { ++count; });
    };

    send_seq(0);
    EXPECT_EQ(reader.stats().lost, 0U);

    // Jump forward by 4: missing sequence 1, 2, 3 -> lost = 3.
    send_seq(4);
    EXPECT_EQ(reader.stats().lost, 3U);

    // Peer restarts, jumping backwards to 0. Must not count as lost.
    send_seq(0);
    EXPECT_EQ(reader.stats().lost, 3U);
}

// --- Session tests ---

class MockSink : public FrameSink {
public:
    bool send(std::span<const std::uint8_t> wire) override
    {
        sent.emplace_back(wire.begin(), wire.end());
        return true;
    }

    std::vector<std::vector<std::uint8_t>> sent;
};

TEST(Session, HandshakeAcceptsThePeerOfferingItsVersionAndSendsTopicInfo)
{
    const std::array<TopicDescription, 2> topics = {
        TopicDescription{
            .id = 10,
            .instances = 1,
            .schema_hash = 0x1122334455667788ULL,
            .name = "imu_sample",
        },
        TopicDescription{
            .id = 20,
            .instances = 2,
            .schema_hash = 0x99AABBCCDDEEFF00ULL,
            .name = "status",
        },
    };
    Session session(topics);
    MockSink sink;

    EXPECT_FALSE(session.connected(1000));
    EXPECT_EQ(session.version(), 0U);

    // Peer sends Hello offering version 1, the current version and 3.
    std::array<std::uint8_t, 4> hello_body{};
    BodyWriter bw(hello_body);
    bw.u8(3);  // count
    bw.u8(1);
    bw.u8(protocol_version);
    bw.u8(3);  // a later version the MCU does not know

    Frame hello_frame{
        .version = 1,
        .type = FrameType::Hello,
        .sequence = 0,
        .body = bw.written(),
    };

    session.handle(hello_frame, 1000, sink);

    EXPECT_TRUE(session.connected(1000));
    EXPECT_EQ(session.version(), protocol_version);

    // Expect 3 frames in sink: 1 HelloReply + 2 TopicInfo.
    ASSERT_EQ(sink.sent.size(), 3U);

    FrameReader reader;
    std::vector<FrameType> types;
    std::uint8_t chosen_ver = 0;
    std::uint16_t topic_count = 0;

    for (const auto& wire : sink.sent) {
        reader.feed(wire, [&](const Frame& f) {
            types.push_back(f.type);
            if (f.type == FrameType::HelloReply) {
                BodyReader br(f.body);
                chosen_ver = br.u8();
                topic_count = br.u16();
            }
        });
    }

    EXPECT_EQ(types, (std::vector<FrameType>{FrameType::HelloReply, FrameType::TopicInfo, FrameType::TopicInfo}));
    EXPECT_EQ(chosen_ver, protocol_version);
    EXPECT_EQ(topic_count, 2U);
}

TEST(Session, HandshakeRejectsPeerWithNoSharedVersion)
{
    Session session;
    MockSink sink;

    // Peer offers version 1 only, which had no parameter access; the MCU offers only the current version.
    std::array<std::uint8_t, 2> hello_body{};
    BodyWriter bw(hello_body);
    bw.u8(1);  // count
    bw.u8(1);

    Frame hello_frame{
        .version = 2,
        .type = FrameType::Hello,
        .sequence = 0,
        .body = bw.written(),
    };

    session.handle(hello_frame, 1000, sink);

    EXPECT_FALSE(session.connected(1000));
    EXPECT_EQ(session.version(), 0U);

    // Expect 1 HelloReply with chosen=0, topic_count=0, and no TopicInfo frames.
    ASSERT_EQ(sink.sent.size(), 1U);
    FrameReader reader;
    reader.feed(sink.sent[0], [&](const Frame& f) {
        EXPECT_EQ(f.type, FrameType::HelloReply);
        BodyReader br(f.body);
        EXPECT_EQ(br.u8(), 0U);   // chosen = 0
        EXPECT_EQ(br.u16(), 0U);  // topics = 0
    });
}

TEST(Session, IgnoresNonHelloFramesBeforeHandshake)
{
    Session session;
    MockSink sink;

    std::array<std::uint8_t, 8> ts_body{};
    BodyWriter bw(ts_body);
    bw.u64(12345ULL);

    Frame ts_frame{
        .version = 1,
        .type = FrameType::TimeSyncRequest,
        .sequence = 0,
        .body = bw.written(),
    };

    session.handle(ts_frame, 1000, sink);
    EXPECT_TRUE(sink.sent.empty());
    EXPECT_FALSE(session.connected(1000));
}

TEST(Session, TimeSyncRepliesWithHostAndMcuTimes)
{
    Session session;
    MockSink sink;

    // Complete handshake first.
    std::array<std::uint8_t, 2> hello_body = {1, protocol_version};
    session.handle({.version = 1, .type = FrameType::Hello, .sequence = 0, .body = hello_body}, 1000, sink);
    sink.sent.clear();

    // Host sends TimeSyncRequest at host time 5000000 us, MCU receives at 200000 us.
    std::array<std::uint8_t, 8> req_body{};
    BodyWriter bw(req_body);
    bw.u64(5'000'000ULL);

    Frame req_frame{
        .version = 1,
        .type = FrameType::TimeSyncRequest,
        .sequence = 1,
        .body = bw.written(),
    };

    session.handle(req_frame, 200'000ULL, sink);

    ASSERT_EQ(sink.sent.size(), 1U);
    FrameReader reader;
    reader.feed(sink.sent[0], [&](const Frame& f) {
        EXPECT_EQ(f.type, FrameType::TimeSyncReply);
        BodyReader br(f.body);
        EXPECT_EQ(br.u64(), 5'000'000ULL);  // host send time
        EXPECT_EQ(br.u64(), 200'000ULL);    // mcu receive time
        EXPECT_EQ(br.u64(), 200'000ULL);    // mcu send time
    });
}

TEST(Session, TopicDataGatedByHandshakeAndPeerTimeout)
{
    constexpr std::uint64_t timeout_us = 1'000'000;
    Session session(schema_topics, timeout_us);
    MockSink sink;

    const std::array<std::uint8_t, 4> sample = {1, 2, 3, 4};

    // 1. Cannot send topic data before handshake.
    EXPECT_FALSE(session.send_topic(1, 0, sample, 100, sink));
    EXPECT_TRUE(sink.sent.empty());

    // 2. Perform handshake at t = 100.
    std::array<std::uint8_t, 2> hello_body = {1, protocol_version};
    session.handle({.version = 1, .type = FrameType::Hello, .sequence = 0, .body = hello_body}, 100, sink);
    sink.sent.clear();

    // 3. Topic data succeeds while connected within timeout.
    EXPECT_TRUE(session.connected(500));
    EXPECT_TRUE(session.send_topic(1, 0, sample, 500, sink));
    EXPECT_EQ(sink.sent.size(), 1U);
    sink.sent.clear();

    // 4. Times out when no peer messages received past timeout.
    EXPECT_FALSE(session.connected(100 + timeout_us + 1));
    EXPECT_FALSE(session.send_topic(1, 0, sample, 100 + timeout_us + 1, sink));
    EXPECT_TRUE(sink.sent.empty());

    // 5. Time sync request from peer resets timeout.
    std::array<std::uint8_t, 8> req_body{};
    BodyWriter bw(req_body);
    bw.u64(12345ULL);
    session.handle({.version = 1, .type = FrameType::TimeSyncRequest, .sequence = 1, .body = bw.written()},
                   2'000'000ULL, sink);
    sink.sent.clear();

    EXPECT_TRUE(session.connected(2'000'000ULL));
    EXPECT_TRUE(session.send_topic(1, 0, sample, 2'500'000ULL, sink));
    EXPECT_EQ(sink.sent.size(), 1U);
}

// --- Parameter access (protocol version 2, ADR-0012) ---

using vk::core::ParamInfo;
using vk::core::ParamResult;
using vk::core::ParamType;
using vk::core::ParamValue;

std::int32_t param_count_value = 3;
float param_gain_value = 0.5F;
std::int32_t param_lock_value = 0;
constexpr std::array<std::int32_t, 2> param_lock_values{0, 1};

std::array<ParamInfo, 3> make_param_table()
{
    return {{
        {.name = "T_COUNT", .hash = vk::core::param_hash("T_COUNT"), .type = ParamType::Int32, .flags = 0,
         .default_value = ParamValue::from_int(3), .min = ParamValue::from_int(0), .max = ParamValue::from_int(10),
         .enum_values = {}, .bitmask = 0, .int_value = &param_count_value, .float_value = nullptr},
        {.name = "T_GAIN", .hash = vk::core::param_hash("T_GAIN"), .type = ParamType::Float, .flags = 0,
         .default_value = ParamValue::from_float(0.5F), .min = ParamValue::from_float(0.0F),
         .max = ParamValue::from_float(2.0F), .enum_values = {}, .bitmask = 0, .int_value = nullptr,
         .float_value = &param_gain_value},
        {.name = "T_LOCK", .hash = vk::core::param_hash("T_LOCK"), .type = ParamType::Int32,
         .flags = vk::core::param_flags::locked_when_armed, .default_value = ParamValue::from_int(0),
         .min = ParamValue::from_int(0), .max = ParamValue::from_int(1), .enum_values = param_lock_values,
         .bitmask = 0, .int_value = &param_lock_value, .float_value = nullptr},
    }};
}

// A received param_value or param_reject.
struct ParamReply {
    FrameType type{};
    std::uint16_t index = 0;
    std::uint16_t count = 0;
    std::uint8_t value_type = 0;
    std::uint32_t bits = 0;
    std::uint8_t flags = 0;
    std::uint8_t reason = 0;
    std::string name;
};

std::vector<ParamReply> param_replies(const std::vector<std::vector<std::uint8_t>>& wires)
{
    std::vector<ParamReply> replies;
    FrameReader reader;
    for (const auto& wire : wires) {
        reader.feed(wire, [&](const Frame& f) {
            BodyReader body(f.body);
            ParamReply reply;
            reply.type = f.type;
            if (f.type == FrameType::ParamValue) {
                reply.index = body.u16();
                reply.count = body.u16();
                reply.value_type = body.u8();
                reply.bits = body.u32();
                reply.flags = body.u8();
            } else if (f.type == FrameType::ParamReject) {
                reply.reason = body.u8();
            } else {
                return;
            }
            const auto name = body.bytes(body.u8());
            reply.name.assign(name.begin(), name.end());
            EXPECT_TRUE(body.ok());
            replies.push_back(reply);
        });
    }
    return replies;
}

class SessionParameters : public ::testing::Test {
protected:
    void SetUp() override
    {
        param_count_value = 3;
        param_gain_value = 0.5F;
        param_lock_value = 0;
        arm(false);
        const std::array<std::uint8_t, 2> hello = {1, protocol_version};
        session.handle({.version = protocol_version, .type = FrameType::Hello, .sequence = 0, .body = hello}, 1000,
                       sink);
        sink.sent.clear();
    }

    void TearDown() override { arm(false); }

    static void arm(bool armed)
    {
        vk::msg::ArmingState state{};
        state.armed = armed;
        vk::core::Publication<vk::msg::ArmingState>{}.publish(state);
    }

    void request(FrameType type, const std::vector<std::uint8_t>& body)
    {
        session.handle({.version = protocol_version, .type = type, .sequence = 1, .body = body}, 2000, sink);
    }

    static std::vector<std::uint8_t> name_body(std::string_view name)
    {
        std::vector<std::uint8_t> body{static_cast<std::uint8_t>(name.size())};
        body.insert(body.end(), name.begin(), name.end());
        return body;
    }

    static std::vector<std::uint8_t> set_body(std::string_view name, ParamValue value)
    {
        std::vector<std::uint8_t> body = name_body(name);
        body.push_back(static_cast<std::uint8_t>(value.type));
        for (int shift = 0; shift < 32; shift += 8) {
            body.push_back(static_cast<std::uint8_t>(value.bits >> static_cast<unsigned>(shift)));
        }
        return body;
    }

    std::array<ParamInfo, 3> table = make_param_table();
    vk::core::Parameters parameters{table};
    Session session{{}, Session::default_peer_timeout_us, {.parameters = &parameters, .set_hash = 0x0123456789ABCDEFULL}};
    MockSink sink;
};

TEST_F(SessionParameters, HelloReplyCarriesTheParameterSetHashAndCount)
{
    Session fresh{{}, Session::default_peer_timeout_us, {.parameters = &parameters, .set_hash = 0x0123456789ABCDEFULL}};
    const std::array<std::uint8_t, 2> hello = {1, protocol_version};
    fresh.handle({.version = protocol_version, .type = FrameType::Hello, .sequence = 0, .body = hello}, 1000, sink);
    ASSERT_EQ(sink.sent.size(), 1U);
    FrameReader reader;
    reader.feed(sink.sent[0], [&](const Frame& f) {
        ASSERT_EQ(f.type, FrameType::HelloReply);
        BodyReader body(f.body);
        EXPECT_EQ(body.u8(), protocol_version);
        EXPECT_EQ(body.u16(), 0U);  // topics
        EXPECT_EQ(body.u64(), 0x0123456789ABCDEFULL);
        EXPECT_EQ(body.u16(), 3U);  // parameters
        EXPECT_TRUE(body.ok());
    });
}

TEST_F(SessionParameters, GetRepliesWithTheValueOrRejectsAnUnknownName)
{
    request(FrameType::ParamGet, name_body("T_GAIN"));
    request(FrameType::ParamGet, name_body("T_NONE"));
    const auto replies = param_replies(sink.sent);
    ASSERT_EQ(replies.size(), 2U);
    EXPECT_EQ(replies[0].type, FrameType::ParamValue);
    EXPECT_EQ(replies[0].name, "T_GAIN");
    EXPECT_EQ(replies[0].index, 1U);
    EXPECT_EQ(replies[0].count, 3U);
    EXPECT_EQ(replies[0].value_type, static_cast<std::uint8_t>(ParamType::Float));
    EXPECT_EQ(replies[0].bits, ParamValue::from_float(0.5F).bits);
    EXPECT_EQ(replies[1].type, FrameType::ParamReject);
    EXPECT_EQ(replies[1].reason, static_cast<std::uint8_t>(ParamResult::UnknownName));
    EXPECT_EQ(replies[1].name, "T_NONE");
}

TEST_F(SessionParameters, SetAppliesAValueAndRepliesWithIt)
{
    request(FrameType::ParamSet, set_body("T_COUNT", ParamValue::from_int(7)));
    const auto replies = param_replies(sink.sent);
    ASSERT_EQ(replies.size(), 1U);
    EXPECT_EQ(replies[0].type, FrameType::ParamValue);
    EXPECT_EQ(replies[0].bits, ParamValue::from_int(7).bits);
    EXPECT_EQ(param_count_value, 7);
}

TEST_F(SessionParameters, SetRejectsWithTheReason)
{
    request(FrameType::ParamSet, set_body("T_NONE", ParamValue::from_int(1)));
    request(FrameType::ParamSet, set_body("T_GAIN", ParamValue::from_int(1)));
    request(FrameType::ParamSet, set_body("T_COUNT", ParamValue::from_int(11)));
    arm(true);
    request(FrameType::ParamSet, set_body("T_LOCK", ParamValue::from_int(1)));
    const auto replies = param_replies(sink.sent);
    ASSERT_EQ(replies.size(), 4U);
    const std::array<ParamResult, 4> reasons{ParamResult::UnknownName, ParamResult::WrongType,
                                             ParamResult::OutOfRange, ParamResult::LockedWhileArmed};
    for (std::size_t i = 0; i < reasons.size(); ++i) {
        EXPECT_EQ(replies[i].type, FrameType::ParamReject) << i;
        EXPECT_EQ(replies[i].reason, static_cast<std::uint8_t>(reasons[i])) << i;
    }
    EXPECT_EQ(param_count_value, 3);
    EXPECT_EQ(param_lock_value, 0);
}

TEST_F(SessionParameters, ListStreamsFromTheFirstIndexAtTheGivenPace)
{
    request(FrameType::ParamListRequest, {1, 0});  // from index 1
    EXPECT_TRUE(sink.sent.empty());
    EXPECT_TRUE(session.listing());
    session.send_pending(2000, sink, 1);
    EXPECT_EQ(sink.sent.size(), 1U);
    session.send_pending(2000, sink, 5);
    EXPECT_FALSE(session.listing());
    const auto replies = param_replies(sink.sent);
    ASSERT_EQ(replies.size(), 2U);
    EXPECT_EQ(replies[0].name, "T_GAIN");
    EXPECT_EQ(replies[1].name, "T_LOCK");
    EXPECT_EQ(replies[1].index, 2U);
}

TEST_F(SessionParameters, ListResendsAFrameTheSinkDropped)
{
    class DroppingSink : public FrameSink {
    public:
        bool send(std::span<const std::uint8_t> wire) override
        {
            if (drop_next) {
                drop_next = false;
                return false;
            }
            sent.emplace_back(wire.begin(), wire.end());
            return true;
        }
        bool drop_next = true;
        std::vector<std::vector<std::uint8_t>> sent;
    } dropping;

    request(FrameType::ParamListRequest, {0, 0});
    session.send_pending(2000, dropping, 10);  // the first frame is dropped; the list waits
    EXPECT_TRUE(dropping.sent.empty());
    session.send_pending(2000, dropping, 10);
    const auto replies = param_replies(dropping.sent);
    ASSERT_EQ(replies.size(), 3U);
    EXPECT_EQ(replies[0].name, "T_COUNT");
}

TEST(Session, WithoutParametersEveryNameIsUnknown)
{
    Session session;
    MockSink sink;
    const std::array<std::uint8_t, 2> hello = {1, protocol_version};
    session.handle({.version = protocol_version, .type = FrameType::Hello, .sequence = 0, .body = hello}, 1000, sink);
    sink.sent.clear();
    const std::array<std::uint8_t, 7> get = {6, 'T', '_', 'G', 'A', 'I', 'N'};
    session.handle({.version = protocol_version, .type = FrameType::ParamGet, .sequence = 1, .body = get}, 1000, sink);
    const auto replies = param_replies(sink.sent);
    ASSERT_EQ(replies.size(), 1U);
    EXPECT_EQ(replies[0].reason, static_cast<std::uint8_t>(ParamResult::UnknownName));
}

}  // namespace
