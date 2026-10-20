#include <vk/link/link.hpp>

#include <array>
#include <cstddef>

namespace vk::link {
namespace {

// Reads per run; bounds the run time when bytes keep arriving.
constexpr int max_reads_per_run = 8;

// A Link's requests come from the Link itself.
ControlAccess with_source(ControlAccess control, std::uint8_t instance)
{
    control.source = instance;
    return control;
}

}  // namespace

Link::Link(std::string_view name, hal::Transport& transport, TopicMap map, const Config& config,
           core::WorkQueue& queue)
    : WorkItem(name, queue), transport_(&transport), map_(map), config_(config),
      session_(schema_topics, config.peer_timeout_us, config.parameters, with_source(config.control, config.instance)),
      sink_(transport), status_publication_(config.instance)
{
}

void Link::start()
{
    rate_scale_ = scale_to_baud(map_, transport_->baud_rate());
    schedule_periodic(config_.interval_us);
}

LinkStats Link::link_stats() const
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
            .handshake_state = handshake_state_,
            .rate_scale = rate_scale_};
}

void Link::run()
{
    const std::uint64_t now = queue().now_us();
    receive(now);
    if (session_.connected(now)) {
        handshake_state_ = HandshakeState::Connected;
    } else {
        handshake_state_ = session_.handshake_done() ? HandshakeState::PeerSilent : HandshakeState::Waiting;
    }
    publish_status(now);
    session_.send_pending(now, sink_, paced_frames_per_run);
    send_topics(now);
}

void Link::publish_status(std::uint64_t now_us)
{
    if (now_us < next_status_us_) {
        return;
    }
    next_status_us_ = now_us + status_interval_us;
    const LinkStats current = link_stats();
    msg::LinkStatus status{};
    status.timestamp = now_us;
    status.frames_sent = current.frames_sent;
    status.bytes_sent = current.bytes_sent;
    status.frames_received = current.frames_received;
    status.bytes_received = current.bytes_received;
    status.drops = current.drops;
    status.crc_errors = current.crc_errors;
    status.malformed = current.malformed;
    status.lost = current.lost;
    status.handshake_state = static_cast<std::uint8_t>(current.handshake_state);
    status.rate_scale = current.rate_scale;
    status_publication_.publish(status);
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
    // Held back, not drained, until the host knows every topic.
    if (handshake_state_ != HandshakeState::Connected || session_.describing()) {
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
