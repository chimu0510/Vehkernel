// Unit tests for the custom protocol: COBS, CRC16, FrameReader/Writer, and Session (ADR-0002).
#include <vk/link/cobs.hpp>
#include <vk/link/crc16.hpp>
#include <vk/link/frame.hpp>
#include <vk/link/session.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
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

TEST(Session, HandshakePicksHighestSharedVersionAndSendsTopicInfo)
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

    // Peer sends Hello offering version 1 and 2.
    std::array<std::uint8_t, 3> hello_body{};
    BodyWriter bw(hello_body);
    bw.u8(2);  // count
    bw.u8(1);  // version 1
    bw.u8(2);  // version 2 (higher than MCU supports)

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
    EXPECT_EQ(chosen_ver, 1U);
    EXPECT_EQ(topic_count, 2U);
}

TEST(Session, HandshakeRejectsPeerWithNoSharedVersion)
{
    Session session;
    MockSink sink;

    // Peer offers version 2 only.
    std::array<std::uint8_t, 2> hello_body{};
    BodyWriter bw(hello_body);
    bw.u8(1);  // count
    bw.u8(2);  // version 2 (MCU only supports 1)

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
    std::array<std::uint8_t, 2> hello_body = {1, 1};
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
    std::array<std::uint8_t, 2> hello_body = {1, 1};
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

}  // namespace
