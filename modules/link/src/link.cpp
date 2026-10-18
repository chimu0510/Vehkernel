#include <vk/link/link.hpp>

#include <array>
#include <cstddef>

namespace vk::link {
namespace {

// Reads per run; bounds the run time when bytes keep arriving.
constexpr int max_reads_per_run = 8;

}  // namespace

Link::Link(std::string_view name, hal::Transport& transport, TopicMap map, const Config& config,
           core::WorkQueue& queue)
    : WorkItem(name, queue), transport_(&transport), map_(map), config_(config),
      session_(schema_topics, config.peer_timeout_us), sink_(transport)
{
}

void Link::start()
{
    schedule_periodic(config_.interval_us);
}

LinkStats Link::stats() const
{
    const ReaderStats& received = reader_.stats();
    return {.frames_sent = sink_.frames,
            .bytes_sent = sink_.bytes,
            .frames_received = received.frames,
            .bytes_received = bytes_received_,
            .drops = sink_.drops,
            .crc_errors = received.crc_errors,
            .malformed = received.malformed,
            .lost = received.lost,
            .connected = connected_};
}

void Link::run()
{
    const std::uint64_t now = queue().now_us();
    receive(now);
    send_topics(now);
}

void Link::receive(std::uint64_t now_us)
{
    for (int i = 0; i < max_reads_per_run; ++i) {
        const std::size_t size = transport_->read(receive_buffer_);
        if (size == 0) {
            return;
        }
        bytes_received_ += size;
        // Frames are handled read by read, so a hello selects the peer it came from.
        reader_.feed(std::span(receive_buffer_).first(size), [&](const Frame& frame) {
            if (frame.type == FrameType::Hello) {
                transport_->select_peer();
            }
            session_.handle(frame, now_us, sink_);
        });
    }
}

void Link::send_topics(std::uint64_t now_us)
{
    connected_ = session_.connected(now_us);
    if (!connected_) {
        return;
    }
    std::array<std::uint8_t, max_message_size> message{};
    for (TopicStream* stream : map_) {
        for (std::size_t size = stream->next(now_us, message); size != 0; size = stream->next(now_us, message)) {
            session_.send_topic(stream->topic_id(), stream->instance(), std::span(message).first(size), now_us,
                                sink_);
        }
    }
}

bool Link::TransportSink::send(std::span<const std::uint8_t> wire)
{
    // Only whole frames go out; a partial one would cost the next frame too.
    if (transport_->write_space() < wire.size() || transport_->write(wire) != wire.size()) {
        ++drops;
        return false;
    }
    ++frames;
    bytes += static_cast<std::uint32_t>(wire.size());
    return true;
}

}  // namespace vk::link
