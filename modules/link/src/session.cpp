// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/link/session.hpp>

#include <array>
#include <limits>

namespace vk::link {

void Session::handle(const Frame& frame, std::uint64_t now_us, FrameSink& sink)
{
    FrameSender out = sender(sink);
    const BodyReader body(frame.body);
    if (frame.type == FrameType::Hello) {
        on_hello(body, out);
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
        on_time_sync(body, now_us, out);
        break;
    case FrameType::ParamListRequest:
    case FrameType::ParamGet:
    case FrameType::ParamSet:
        parameters_.handle(frame.type, body, out);
        break;
    case FrameType::ModeRequest:
    case FrameType::ArmingRequest:
    case FrameType::EstopRequest:
    case FrameType::BringupRun:
    case FrameType::RebootRequest:
        control_.handle(frame.type, body, now_us);
        break;
    case FrameType::SimActuatorPower:
    case FrameType::SimUserKey:
    case FrameType::SimGamepad:
        if (sim_frames_ != nullptr) {
            sim_frames_->handle(frame.type, body, out);
        }
        break;
    default:
        break;  // Frames only the MCU sends, or not meant for a session.
    }
}

bool Session::connected(std::uint64_t now_us) const
{
    return handshake_done_ && now_us - last_heard_us_ <= peer_timeout_us_;
}

void Session::on_hello(BodyReader body, FrameSender& out)
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
    parameters_.stop_list();  // A new peer starts afresh.
    // send_pending() describes the topics; a burst here would overflow the transport.
    info_next_ = handshake_done_ ? 0 : std::numeric_limits<std::size_t>::max();

    std::array<std::uint8_t, 13> reply{};
    BodyWriter writer(reply);
    writer.u8(chosen);
    writer.u16(static_cast<std::uint16_t>(handshake_done_ ? topics_.size() : 0));
    writer.u64(handshake_done_ ? parameters_.set_hash() : 0);
    writer.u16(static_cast<std::uint16_t>(handshake_done_ ? parameters_.count() : 0));
    out.send(FrameType::HelloReply, writer.written());
}

bool Session::send_topic_info(const TopicDescription& topic, FrameSender& out)
{
    std::array<std::uint8_t, max_body_size> info{};
    BodyWriter writer(info);
    writer.u16(topic.id);
    writer.u8(topic.instances);
    writer.u64(topic.schema_hash);
    writer.u8(static_cast<std::uint8_t>(topic.name.size()));
    writer.bytes({reinterpret_cast<const std::uint8_t*>(topic.name.data()), topic.name.size()});
    return writer.ok() && out.send(FrameType::TopicInfo, writer.written());
}

void Session::on_time_sync(BodyReader body, std::uint64_t now_us, FrameSender& out)
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
    out.send(FrameType::TimeSyncReply, writer.written());
}

void Session::send_answers(std::uint64_t now_us, FrameSink& sink)
{
    FrameSender out = sender(sink);
    control_.send_answers(now_us, out);
}

void Session::source_lost(std::uint64_t now_us)
{
    control_.source_lost(now_us);
}

bool Session::send_bringup_result(const msg::BringupStatus& status, FrameSink& sink)
{
    FrameSender out = sender(sink);
    return control_.send_bringup_result(status, out);
}

void Session::send_pending(std::uint64_t now_us, FrameSink& sink, std::size_t max_frames)
{
    FrameSender out = sender(sink);
    for (std::size_t sent = 0; sent < max_frames && (describing() || listing()) && connected(now_us); ++sent) {
        if (describing()) {
            if (!send_topic_info(topics_[info_next_], out)) {
                return;  // Dropped: the same topic goes again next time.
            }
            ++info_next_;
            continue;
        }
        if (!parameters_.send_next(out)) {
            return;  // Dropped: the same parameter goes again next time.
        }
    }
}

bool Session::send_topic(std::uint16_t topic_id, std::uint8_t instance, std::span<const std::uint8_t> message,
                         std::uint64_t now_us, FrameSink& sink)
{
    if (!connected(now_us) || describing()) {
        return false;
    }
    std::array<std::uint8_t, max_body_size> body{};
    BodyWriter writer(body);
    writer.u16(topic_id);
    writer.u8(instance);
    writer.bytes(message);
    FrameSender out = sender(sink);
    return writer.ok() && out.send(FrameType::TopicData, writer.written());
}

}  // namespace vk::link
