#include <vk/link/session.hpp>

#include <algorithm>

namespace vk::link {

void Session::handle(const Frame& frame, std::uint64_t now_us, FrameSink& sink)
{
    if (frame.type == FrameType::Hello) {
        on_hello(BodyReader(frame.body), sink);
        if (handshake_done_) {
            last_heard_us_ = now_us;
        }
        return;
    }
    if (!handshake_done_) {
        return;  // Nothing but hello counts before the handshake.
    }
    last_heard_us_ = now_us;
    if (frame.type == FrameType::TimeSyncRequest) {
        on_time_sync(BodyReader(frame.body), now_us, sink);
    }
}

bool Session::connected(std::uint64_t now_us) const
{
    return handshake_done_ && now_us - last_heard_us_ <= peer_timeout_us_;
}

void Session::on_hello(BodyReader body, FrameSink& sink)
{
    const std::uint8_t count = body.u8();
    std::uint8_t chosen = 0;
    for (std::uint8_t i = 0; i < count; ++i) {
        const std::uint8_t offered = body.u8();
        if (offered <= protocol_version) {
            chosen = std::max(chosen, offered);
        }
    }
    if (!body.ok()) {
        return;
    }
    version_ = chosen;
    handshake_done_ = chosen != 0;

    std::array<std::uint8_t, 3> reply{};
    BodyWriter writer(reply);
    writer.u8(chosen);
    writer.u16(static_cast<std::uint16_t>(handshake_done_ ? topics_.size() : 0));
    send(FrameType::HelloReply, writer.written(), sink);
    if (!handshake_done_) {
        return;
    }
    for (const TopicDescription& topic : topics_) {
        std::array<std::uint8_t, max_body_size> info{};
        BodyWriter out(info);
        out.u16(topic.id);
        out.u8(topic.instances);
        out.u64(topic.schema_hash);
        out.u8(static_cast<std::uint8_t>(topic.name.size()));
        out.bytes({reinterpret_cast<const std::uint8_t*>(topic.name.data()), topic.name.size()});
        send(FrameType::TopicInfo, out.written(), sink);
    }
}

void Session::on_time_sync(BodyReader body, std::uint64_t now_us, FrameSink& sink)
{
    const std::uint64_t host_send = body.u64();
    if (!body.ok()) {
        return;
    }
    std::array<std::uint8_t, 24> reply{};
    BodyWriter writer(reply);
    writer.u64(host_send);
    writer.u64(now_us);  // received
    writer.u64(now_us);  // sent: the reply leaves within the same run
    send(FrameType::TimeSyncReply, writer.written(), sink);
}

bool Session::send_topic(std::uint16_t topic_id, std::uint8_t instance, std::span<const std::uint8_t> message,
                         std::uint64_t now_us, FrameSink& sink)
{
    if (!connected(now_us)) {
        return false;
    }
    std::array<std::uint8_t, max_body_size> body{};
    BodyWriter writer(body);
    writer.u16(topic_id);
    writer.u8(instance);
    writer.bytes(message);
    return writer.ok() && send(FrameType::TopicData, writer.written(), sink);
}

bool Session::send(FrameType type, std::span<const std::uint8_t> body, FrameSink& sink)
{
    std::array<std::uint8_t, max_wire_size> wire{};
    const std::size_t size = writer_.write(type, body, wire);
    if (size == 0 || !sink.send(std::span(wire).first(size))) {
        return false;
    }
    ++frames_sent_;
    return true;
}

}  // namespace vk::link
