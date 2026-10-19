#include <vk/link/session.hpp>

#include <array>
#include <limits>
#include <optional>
#include <string_view>

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
    switch (frame.type) {
    case FrameType::TimeSyncRequest:
        on_time_sync(BodyReader(frame.body), now_us, sink);
        break;
    case FrameType::ParamListRequest:
        on_param_list(BodyReader(frame.body));
        break;
    case FrameType::ParamGet:
        on_param_get(BodyReader(frame.body), sink);
        break;
    case FrameType::ParamSet:
        on_param_set(BodyReader(frame.body), sink);
        break;
    default:
        break;  // Frames only the MCU sends, or not meant for a session.
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
        if (body.u8() == protocol_version) {
            chosen = protocol_version;
        }
    }
    if (!body.ok()) {
        return;
    }
    version_ = chosen;
    handshake_done_ = chosen != 0;
    list_next_ = std::numeric_limits<std::size_t>::max();  // A new peer starts afresh.

    std::array<std::uint8_t, 13> reply{};
    BodyWriter writer(reply);
    writer.u8(chosen);
    writer.u16(static_cast<std::uint16_t>(handshake_done_ ? topics_.size() : 0));
    writer.u64(handshake_done_ ? parameters_.set_hash : 0);
    writer.u16(static_cast<std::uint16_t>(handshake_done_ ? parameter_count() : 0));
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

namespace {

// Reads a name: a length and that many characters.
std::string_view read_name(BodyReader& body)
{
    const auto bytes = body.bytes(body.u8());
    return {reinterpret_cast<const char*>(bytes.data()), bytes.size()};
}

void write_name(BodyWriter& out, std::string_view name)
{
    out.u8(static_cast<std::uint8_t>(name.size()));
    out.bytes({reinterpret_cast<const std::uint8_t*>(name.data()), name.size()});
}

}  // namespace

void Session::on_param_list(BodyReader body)
{
    const std::uint16_t first = body.u16();
    if (body.ok()) {
        list_next_ = first;
    }
}

void Session::on_param_get(BodyReader body, FrameSink& sink)
{
    const std::string_view name = read_name(body);
    if (!body.ok()) {
        return;
    }
    const auto index = parameter_count() == 0 ? std::nullopt : parameters_.parameters->find(name);
    if (!index) {
        send_param_reject(core::ParamResult::UnknownName, name, sink);
        return;
    }
    send_param_value(*index, sink);
}

void Session::on_param_set(BodyReader body, FrameSink& sink)
{
    const std::string_view name = read_name(body);
    const std::uint8_t type = body.u8();
    const std::uint32_t bits = body.u32();
    if (!body.ok()) {
        return;
    }
    const auto index = parameter_count() == 0 ? std::nullopt : parameters_.parameters->find(name);
    if (!index) {
        send_param_reject(core::ParamResult::UnknownName, name, sink);
        return;
    }
    const core::ParamValue value{.type = static_cast<core::ParamType>(type), .bits = bits};
    const core::ParamResult result = parameters_.parameters->set(*index, value);
    if (result != core::ParamResult::Ok) {
        send_param_reject(result, name, sink);
        return;
    }
    send_param_value(*index, sink);
}

bool Session::send_param_value(std::size_t index, FrameSink& sink)
{
    const core::ParamInfo& info = parameters_.parameters->info(index);
    const core::ParamValue value = parameters_.parameters->get(index);
    std::array<std::uint8_t, max_body_size> body{};
    BodyWriter out(body);
    out.u16(static_cast<std::uint16_t>(index));
    out.u16(static_cast<std::uint16_t>(parameter_count()));
    out.u8(static_cast<std::uint8_t>(value.type));
    out.u32(value.bits);
    out.u8(0);  // flags
    write_name(out, info.name);
    return out.ok() && send(FrameType::ParamValue, out.written(), sink);
}

void Session::send_param_reject(core::ParamResult reason, std::string_view name, FrameSink& sink)
{
    std::array<std::uint8_t, max_body_size> body{};
    BodyWriter out(body);
    out.u8(static_cast<std::uint8_t>(reason));
    write_name(out, name);
    if (out.ok()) {
        send(FrameType::ParamReject, out.written(), sink);
    }
}

void Session::send_pending(std::uint64_t now_us, FrameSink& sink, std::size_t max_frames)
{
    for (std::size_t sent = 0; sent < max_frames && listing() && connected(now_us); ++sent) {
        if (!send_param_value(list_next_, sink)) {
            return;  // Dropped: the same parameter goes again next time.
        }
        ++list_next_;
    }
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
    if (size == 0) {
        return false;
    }
    if (!sink.send(std::span(wire).first(size))) {
        writer_.retract();
        return false;
    }
    ++frames_sent_;
    return true;
}

}  // namespace vk::link
