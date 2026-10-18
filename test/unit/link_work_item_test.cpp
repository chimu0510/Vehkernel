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

    std::size_t space = std::numeric_limits<std::size_t>::max();
    std::vector<std::uint8_t> written;
    std::vector<std::uint8_t> incoming;
    bool peer_selected = false;
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
    EXPECT_FALSE(link.stats().connected);
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
    const auto frames = host.receive();
    ASSERT_FALSE(frames.empty());
    EXPECT_EQ(frames.front().type, FrameType::HelloReply);
    EXPECT_EQ(frames.size(), 1 + vk::link::schema_topics.size());
    EXPECT_TRUE(link.stats().connected);
}

TEST_F(LinkTest, SendsEverySampleOfAQueuedTopicInOrder)
{
    using Topic = Sample<2, 4>;
    TopicStreamOf<Topic> stream(vk::link::every_sample);
    const std::array<TopicStream*, 1> map{&stream};
    Link link("link_test", transport, map, {}, queue);
    link.start();
    host.send_hello();
    step();
    host.receive();

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
    host.send_hello();
    step();
    host.receive();

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
    host.send_hello();
    step();
    host.receive();

    vk::core::Publication<Topic> publication;
    for (int i = 0; i < 10; ++i) {
        publication.publish({.timestamp = fake_now, .value = 7});
        step();
    }
    // Sent until the timeout (20 ms, four runs), then nothing.
    EXPECT_EQ(host.receive_values(Topic::topic_id).size(), 4U);
    EXPECT_FALSE(link.stats().connected);
}

}  // namespace
