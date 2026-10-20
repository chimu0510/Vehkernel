// The Link work item over a fake transport (ADR-0002, ADR-0008): the
// handshake, topic map rates, every-sample queued topics and drops. The
// development host's side is played with the protocol's own frame reader and
// writer; the work queue runs on a fake clock.
#include <vk/core/bus.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/hal/transport.hpp>
#include <vk/link/frame.hpp>
#include <vk/link/link.hpp>
#include <vk/link/topic_map.hpp>
#include <vk/msg/link_status.hpp>
#include <vk/msg/wire.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <string_view>
#include <vector>

namespace {

using vk::link::FrameType;
using vk::link::Link;
using vk::link::TopicStream;
using vk::link::TopicStreamOf;

std::uint64_t fake_now = 0;

std::uint64_t fake_clock() noexcept
{
    return fake_now;
}

// A message type of its own per test, so no test sees another's publications.
template <int Tag, std::size_t QueueLength>
struct Sample {
    std::uint64_t timestamp{};
    std::uint32_t value{};

    static constexpr std::string_view topic_name = "link_test_sample";
    static constexpr std::uint64_t schema_hash = 0;
    static constexpr std::uint16_t topic_id = 100 + Tag;
    static constexpr std::size_t queue_length = QueueLength;
    static constexpr std::size_t instances = 1;
    static constexpr float nominal_rate_hz = 0.0F;
    static constexpr std::size_t encoded_size = 12;

    [[nodiscard]] std::size_t encode(std::span<std::uint8_t> out) const
    {
        if (out.size() < encoded_size) {
            return 0;
        }
        std::size_t at = 0;
        vk::msg::wire::put(out, at, timestamp);
        vk::msg::wire::put(out, at, value);
        return at;
    }
};

class FakeTransport final : public vk::hal::Transport {
public:
    FakeTransport() = default;
    FakeTransport(const FakeTransport&) = delete;
    FakeTransport& operator=(const FakeTransport&) = delete;
    FakeTransport(FakeTransport&&) = delete;
    FakeTransport& operator=(FakeTransport&&) = delete;
    ~FakeTransport() = default;

    [[nodiscard]] std::size_t write_space() const override { return space; }

    std::size_t write(std::span<const std::uint8_t> data) override
    {
        const std::size_t taken = std::min(space, data.size());
        space -= taken;
        written.insert(written.end(), data.begin(), data.begin() + static_cast<std::ptrdiff_t>(taken));
        return taken;
    }

    std::size_t read(std::span<std::uint8_t> out) override
    {
        const std::size_t size = std::min(out.size(), incoming.size());
        std::copy_n(incoming.begin(), size, out.begin());
        incoming.erase(incoming.begin(), incoming.begin() + static_cast<std::ptrdiff_t>(size));
        return size;
    }

    void select_peer() override { peer_selected = true; }
    [[nodiscard]] std::uint32_t baud_rate() const override { return baud; }

    std::size_t space = std::numeric_limits<std::size_t>::max();
    std::vector<std::uint8_t> written;
    std::vector<std::uint8_t> incoming;
    bool peer_selected = false;
    std::uint32_t baud = 0;
};

struct ReceivedFrame {
    FrameType type;
    std::uint16_t sequence;
    std::vector<std::uint8_t> body;
};

// The development host's side of the Link.
class Host {
public:
    explicit Host(FakeTransport& transport) : transport_(&transport) {}

    void send(FrameType type, std::span<const std::uint8_t> body)
    {
        std::array<std::uint8_t, vk::link::max_wire_size> wire{};
        const std::size_t size = writer_.write(type, body, wire);
        transport_->incoming.insert(transport_->incoming.end(), wire.begin(),
                                    wire.begin() + static_cast<std::ptrdiff_t>(size));
    }

    // A frame lost on the way: numbered, never delivered.
    void lose_one()
    {
        std::array<std::uint8_t, vk::link::max_wire_size> wire{};
        writer_.write(FrameType::TimeSyncRequest, std::array<std::uint8_t, 8>{}, wire);
    }

    // A time sync request with one byte changed on the way.
    void send_corrupted()
    {
        std::array<std::uint8_t, vk::link::max_wire_size> wire{};
        const std::size_t size = writer_.write(FrameType::TimeSyncRequest, std::array<std::uint8_t, 8>{}, wire);
        wire[3] = wire[3] == 0x55 ? 0x56 : 0x55;  // never 0, so the frame keeps its length
        transport_->incoming.insert(transport_->incoming.end(), wire.begin(),
                                    wire.begin() + static_cast<std::ptrdiff_t>(size));
    }

    void send_time_sync()
    {
        send(FrameType::TimeSyncRequest, std::array<std::uint8_t, 8>{});
    }

    void send_hello()
    {
        const std::array<std::uint8_t, 2> body{1, vk::link::protocol_version};
        send(FrameType::Hello, body);
    }

    // Every frame the Link wrote since the last call.
    std::vector<ReceivedFrame> receive()
    {
        std::vector<ReceivedFrame> frames;
        reader_.feed(transport_->written, [&](const vk::link::Frame& frame) {
            frames.push_back({frame.type, frame.sequence, {frame.body.begin(), frame.body.end()}});
        });
        transport_->written.clear();
        return frames;
    }

    // The values of every topic data frame of `topic_id` since the last call.
    std::vector<std::uint32_t> receive_values(std::uint16_t topic_id)
    {
        std::vector<std::uint32_t> values;
        for (const auto& frame : receive()) {
            if (frame.type != FrameType::TopicData) {
                continue;
            }
            vk::link::BodyReader body(frame.body);
            if (body.u16() != topic_id) {
                continue;
            }
            body.u8();  // instance
            body.u64();  // timestamp
            values.push_back(static_cast<std::uint32_t>(body.u16() | (static_cast<std::uint32_t>(body.u16()) << 16U)));
        }
        return values;
    }

    [[nodiscard]] const vk::link::ReaderStats& stats() const { return reader_.stats(); }

private:
    FakeTransport* transport_;
    vk::link::FrameWriter writer_;
    vk::link::FrameReader reader_;
};

class LinkTest : public ::testing::Test {
protected:
    void SetUp() override { fake_now = 1'000'000; }

    // Runs the Link once, one interval later.
    void step()
    {
        fake_now += Link::default_interval_us;
        queue.process();
    }

    // Says hello and runs the Link until it has described every topic.
    void connect()
    {
        host.send_hello();
        const std::size_t topics = vk::link::schema_topics.size();
        for (std::size_t run = 0; run < (topics + Link::paced_frames_per_run - 1) / Link::paced_frames_per_run; ++run) {
            step();
        }
        host.receive();
    }

    vk::core::WorkQueue queue{"wq:test", 1, &fake_clock};
    FakeTransport transport;
    Host host{transport};
};

TEST_F(LinkTest, SendsNoTopicDataBeforeTheHandshake)
{
    using Topic = Sample<0, 4>;
    TopicStreamOf<Topic> stream(vk::link::every_sample);
    const std::array<TopicStream*, 1> map{&stream};
    Link link("link_test", transport, map, {}, queue);
    link.start();

    vk::core::Publication<Topic>().publish({.timestamp = fake_now, .value = 1});
    step();
    EXPECT_TRUE(transport.written.empty());
    EXPECT_EQ(link.link_stats().handshake_state, vk::link::HandshakeState::Waiting);
}

TEST_F(LinkTest, AnswersHelloToThePeerItCameFrom)
{
    TopicStreamOf<Sample<1, 1>> stream(vk::link::every_sample);
    const std::array<TopicStream*, 1> map{&stream};
    Link link("link_test", transport, map, {}, queue);
    link.start();

    host.send_hello();
    step();
    EXPECT_TRUE(transport.peer_selected);
    auto frames = host.receive();
    ASSERT_FALSE(frames.empty());
    EXPECT_EQ(frames.front().type, FrameType::HelloReply);
    EXPECT_EQ(frames.size(), 1 + Link::paced_frames_per_run);  // the topic_info frames are paced
    EXPECT_EQ(link.link_stats().handshake_state, vk::link::HandshakeState::Connected);
    while (frames.size() < 1 + vk::link::schema_topics.size()) {
        step();
        const auto more = host.receive();
        frames.insert(frames.end(), more.begin(), more.end());
    }
    EXPECT_EQ(frames.size(), 1 + vk::link::schema_topics.size());
}

TEST_F(LinkTest, HandshakeThroughASmallTransmitBufferDropsNothing)
{
    // A topic due every run, and a transmit buffer that takes a few frames per run.
    using Topic = Sample<7, 1>;
    TopicStreamOf<Topic> stream(vk::link::every_sample);
    const std::array<TopicStream*, 1> map{&stream};
    Link link("link_test", transport, map, {}, queue);
    link.start();
    vk::core::Publication<Topic> publication;

    host.send_hello();
    std::vector<ReceivedFrame> frames;
    for (int run = 0; run < 10; ++run) {
        transport.space = 256;  // what the wire drained since the last run
        publication.publish({.timestamp = fake_now, .value = 1});
        step();
        const auto more = host.receive();
        frames.insert(frames.end(), more.begin(), more.end());
    }
    EXPECT_EQ(link.link_stats().drops, 0U);
    EXPECT_EQ(host.stats().lost, 0U);

    // hello_reply, every topic_info, and only then topic data.
    ASSERT_GT(frames.size(), 1 + vk::link::schema_topics.size());
    EXPECT_EQ(frames.front().type, FrameType::HelloReply);
    for (std::size_t i = 1; i < frames.size(); ++i) {
        const FrameType expected = i <= vk::link::schema_topics.size() ? FrameType::TopicInfo : FrameType::TopicData;
        EXPECT_EQ(frames[i].type, expected) << i;
    }
}

TEST_F(LinkTest, SendsEverySampleOfAQueuedTopicInOrder)
{
    using Topic = Sample<2, 4>;
    TopicStreamOf<Topic> stream(vk::link::every_sample);
    const std::array<TopicStream*, 1> map{&stream};
    Link link("link_test", transport, map, {}, queue);
    link.start();
    connect();

    vk::core::Publication<Topic> publication;
    std::vector<std::uint32_t> received;
    for (std::uint32_t value = 1; value <= 30; ++value) {
        publication.publish({.timestamp = fake_now, .value = value});
        if (value % 3 == 0) {
            // Three samples per run, as when the Link runs slower than the IMU.
            step();
            const auto values = host.receive_values(Topic::topic_id);
            received.insert(received.end(), values.begin(), values.end());
        }
    }
    std::vector<std::uint32_t> expected(30);
    for (std::uint32_t i = 0; i < expected.size(); ++i) {
        expected[i] = i + 1;
    }
    EXPECT_EQ(received, expected);
    EXPECT_EQ(host.stats().lost, 0U);
}

TEST_F(LinkTest, SendsALatestValueTopicAtMostAtItsRate)
{
    using Topic = Sample<3, 1>;
    TopicStreamOf<Topic> stream(vk::link::MaxRate{10.0F});
    const std::array<TopicStream*, 1> map{&stream};
    Link link("link_test", transport, map, {}, queue);
    link.start();
    connect();

    // Published at 200 Hz for one second, sent at 10 Hz: the newest each time.
    vk::core::Publication<Topic> publication;
    std::vector<std::uint32_t> received;
    for (std::uint32_t value = 1; value <= 200; ++value) {
        publication.publish({.timestamp = fake_now, .value = value});
        step();
        const auto values = host.receive_values(Topic::topic_id);
        received.insert(received.end(), values.begin(), values.end());
    }
    ASSERT_EQ(received.size(), 10U);
    for (std::size_t i = 0; i < received.size(); ++i) {
        EXPECT_EQ(received[i], 1 + 20 * i);
    }
}

TEST_F(LinkTest, StopsSendingWhenThePeerFallsSilent)
{
    using Topic = Sample<4, 1>;
    TopicStreamOf<Topic> stream(vk::link::every_sample);
    const std::array<TopicStream*, 1> map{&stream};
    Link link("link_test", transport, map, {.peer_timeout_us = 20'000}, queue);
    link.start();
    connect();
    host.send_time_sync();  // heard in the first run below

    vk::core::Publication<Topic> publication;
    for (int i = 0; i < 10; ++i) {
        publication.publish({.timestamp = fake_now, .value = 7});
        step();
    }
    // Sent in the run the peer was heard and until the timeout (20 ms, four runs), then nothing.
    EXPECT_EQ(host.receive_values(Topic::topic_id).size(), 5U);
    EXPECT_EQ(link.link_stats().handshake_state, vk::link::HandshakeState::PeerSilent);
}

TEST_F(LinkTest, DropsAndCountsFramesTheTransportCannotTakeWithoutBlocking)
{
    using Topic = Sample<5, 4>;
    TopicStreamOf<Topic> stream(vk::link::every_sample);
    const std::array<TopicStream*, 1> map{&stream};
    Link link("link_test", transport, map, {}, queue);
    link.start();
    connect();

    // The transport takes nothing: every frame is dropped, and the run returns.
    vk::core::Publication<Topic> publication;
    transport.space = 0;
    for (std::uint32_t value = 1; value <= 3; ++value) {
        publication.publish({.timestamp = fake_now, .value = value});
    }
    step();
    EXPECT_TRUE(transport.written.empty());
    EXPECT_EQ(link.link_stats().drops, 3U);
    EXPECT_EQ(link.link_stats().frames_sent, 1 + vk::link::schema_topics.size());  // the handshake only
    EXPECT_EQ(link.stats().overruns, 0U);

    // Dropped frames give their sequence numbers back: the host loses none.
    transport.space = std::numeric_limits<std::size_t>::max();
    publication.publish({.timestamp = fake_now, .value = 4});
    step();
    EXPECT_EQ(host.receive_values(Topic::topic_id), std::vector<std::uint32_t>{4});
    EXPECT_EQ(host.stats().lost, 0U);
}

TEST_F(LinkTest, CountsCrcErrorsAndSequenceGapsInLinkStatus)
{
    TopicStreamOf<Sample<6, 1>> stream(vk::link::every_sample);
    const std::array<TopicStream*, 1> map{&stream};
    Link link("link_test", transport, map, {.instance = 1}, queue);
    link.start();
    host.send_hello();
    // Until every topic_info is out, so link_status sees every frame sent.
    for (std::size_t run = 0; run <= vk::link::schema_topics.size() / Link::paced_frames_per_run; ++run) {
        step();
    }

    host.send_corrupted();
    host.lose_one();
    host.send_time_sync();
    step();
    fake_now += Link::status_interval_us;  // the next link_status is due
    step();

    vk::core::Subscription<vk::msg::LinkStatus> subscription(1);
    vk::msg::LinkStatus status{};
    while (subscription.copy(status)) {
        // The newest is the one that counts.
    }
    EXPECT_EQ(status.timestamp, fake_now);
    EXPECT_EQ(status.crc_errors, 1U);
    EXPECT_EQ(status.lost, 2U);  // the corrupted frame and the one lost on the way
    EXPECT_EQ(status.frames_received, 2U);  // hello and the time sync request
    EXPECT_EQ(status.handshake_state, static_cast<std::uint8_t>(vk::link::HandshakeState::Connected));
    EXPECT_EQ(status.frames_sent, link.link_stats().frames_sent);
    EXPECT_GT(status.bytes_sent, 0U);
    EXPECT_GT(status.bytes_received, 0U);
}

// --- Topic rates scaled to the baud rate (ADR-0002, spec 0006) ---

// A large message, to make a slow stream count.
template <int Tag>
struct Large {
    std::uint64_t timestamp{};

    static constexpr std::string_view topic_name = "link_test_large";
    static constexpr std::uint64_t schema_hash = 0;
    static constexpr std::uint16_t topic_id = 200 + Tag;
    static constexpr std::size_t queue_length = 1;
    static constexpr std::size_t instances = 1;
    static constexpr float nominal_rate_hz = 0.0F;
    static constexpr std::size_t encoded_size = 200;

    [[nodiscard]] std::size_t encode(std::span<std::uint8_t> out) const
    {
        if (out.size() < encoded_size) {
            return 0;
        }
        std::fill_n(out.begin(), encoded_size, std::uint8_t{1});
        return encoded_size;
    }
};

// A 12-byte message on the wire: 4 header, 3 topic, 12 message and 2 CRC
// bytes, one COBS overhead byte and the delimiter.
constexpr float small_frame_bytes = 23.0F;

TEST(TopicMapLoad, CountsEachFrameWithItsOverhead)
{
    TopicStreamOf<Sample<20, 1>> stream(vk::link::MaxRate{10.0F});
    const std::array<TopicStream*, 1> map{&stream};
    EXPECT_FLOAT_EQ(vk::link::topic_map_load(map), 10.0F * small_frame_bytes);
}

TEST(TopicMapScaling, LeavesAMapThatFitsAlone)
{
    TopicStreamOf<Sample<21, 4>> fast(vk::link::every_sample, 0, 200.0F);
    TopicStreamOf<Sample<22, 1>> slow(vk::link::MaxRate{50.0F});
    const std::array<TopicStream*, 2> map{&fast, &slow};
    EXPECT_EQ(vk::link::scale_to_baud(map, 921'600), 100);
    EXPECT_EQ(vk::link::scale_to_baud(map, 0), 100);  // no baud rate, as UDP
    EXPECT_FLOAT_EQ(fast.rate_hz(), 200.0F);
    EXPECT_FLOAT_EQ(slow.rate_hz(), 50.0F);
}

TEST(TopicMapScaling, ScalesEveryStreamFasterThanOneHertzByOneFactor)
{
    TopicStreamOf<Sample<23, 4>> fast(vk::link::every_sample, 0, 1000.0F);
    TopicStreamOf<Sample<24, 1>> medium(vk::link::MaxRate{100.0F});
    TopicStreamOf<Sample<25, 1>> status(vk::link::MaxRate{1.0F});
    const std::array<TopicStream*, 3> map{&fast, &medium, &status};
    const std::uint8_t scale = vk::link::scale_to_baud(map, 57'600);
    const float capacity = 57'600.0F / 10.0F * vk::link::max_port_load;
    EXPECT_LT(scale, 100);
    EXPECT_LE(vk::link::topic_map_load(map), capacity * 1.001F);
    EXPECT_NEAR(fast.rate_hz() / 1000.0F, medium.rate_hz() / 100.0F, 1e-3F);  // one factor
    EXPECT_NEAR(fast.rate_hz() / 1000.0F, scale / 100.0F, 0.011F);
    EXPECT_FLOAT_EQ(status.rate_hz(), 1.0F);
}

TEST(TopicMapScaling, HoldsAStreamAtOneHertzRatherThanBelow)
{
    TopicStreamOf<Sample<26, 4>> fast(vk::link::every_sample, 0, 1000.0F);
    TopicStreamOf<Large<0>> large(vk::link::MaxRate{2.0F});
    const std::array<TopicStream*, 2> map{&fast, &large};
    vk::link::scale_to_baud(map, 9'600);
    EXPECT_FLOAT_EQ(large.rate_hz(), 1.0F);
    EXPECT_LE(vk::link::topic_map_load(map), 9'600.0F / 10.0F * vk::link::max_port_load * 1.001F);
    EXPECT_GT(fast.rate_hz(), 1.0F);
}

TEST(TopicMapScaling, AnEverySampleStreamSendsOnlyTheNewestSampleOfEachInterval)
{
    TopicStreamOf<Sample<27, 8>> stream(vk::link::every_sample, 0, 1000.0F);
    const std::array<TopicStream*, 1> map{&stream};
    vk::link::scale_to_baud(map, 57'600);
    vk::core::Publication<Sample<27, 8>> publication;
    for (std::uint32_t value = 1; value <= 5; ++value) {
        publication.publish({.timestamp = 0, .value = value});
    }
    std::array<std::uint8_t, 64> out{};
    EXPECT_GT(stream.next(1'000'000, out), 0U);
    EXPECT_EQ(stream.next(1'000'000, out), 0U);
}

TEST(TopicMapScaling, TheDefaultMapFitsEitherRateItIsScaledTo)
{
    const vk::link::TopicMap map = vk::link::default_topic_map(200.0F);
    // Checked by its properties only: later milestones add topics.
    EXPECT_EQ(vk::link::scale_to_baud(map, 921'600), 100);
    EXPECT_LT(vk::link::scale_to_baud(map, 57'600), 100);
    EXPECT_LE(vk::link::topic_map_load(map), 57'600.0F / 10.0F * vk::link::max_port_load * 1.001F);
    for (const TopicStream* stream : map) {
        EXPECT_GE(stream->rate_hz(), std::min(1.0F, stream->nominal_hz()) * 0.999F);
    }
}

TEST_F(LinkTest, ASerialLinkScaledToItsBaudRateDropsNoFrame)
{
    TopicStreamOf<Sample<28, 8>> stream(vk::link::every_sample, 0, 1000.0F);
    const std::array<TopicStream*, 1> map{&stream};
    transport.baud = 57'600;
    Link link("link_test", transport, map, {}, queue);
    link.start();
    connect();
    host.receive();
    const std::uint32_t drops_after_handshake = link.link_stats().drops;

    // The port carries baud / 10 bytes a second; each run may take what has
    // accumulated since the last one.
    vk::core::Publication<Sample<28, 8>> publication;
    float budget = 0.0F;
    for (int run = 0; run < 200; ++run) {
        for (std::uint32_t sample = 0; sample < 5; ++sample) {  // 1000 samples a second
            publication.publish({.timestamp = fake_now, .value = sample});
        }
        budget += 57'600.0F / 10.0F * static_cast<float>(Link::default_interval_us) * 1e-6F;
        transport.space = static_cast<std::size_t>(budget);
        step();
        budget -= static_cast<float>(transport.written.size());
        host.receive();
    }
    EXPECT_EQ(link.link_stats().drops, drops_after_handshake);
    EXPECT_LT(link.link_stats().rate_scale, 100);
}

}  // namespace
