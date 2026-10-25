// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/link/control_handler.hpp>

#include <array>
#include <cstddef>
#include <cstdint>

namespace vk::link {

ControlHandler::ControlHandler(ControlAccess access) : access_(access)
{
    msg::VehicleCommandAck stale{};
    while (acks_.copy(stale)) {
    }
}

void ControlHandler::handle(FrameType type, BodyReader body, std::uint64_t now_us)
{
    const std::uint8_t argument = body.u8();
    if (!body.ok() || !access_.enabled) {
        return;
    }
    switch (type) {
    case FrameType::ModeRequest:
        request(core::Command::Mode, argument, now_us);
        break;
    case FrameType::ArmingRequest:
        request(core::Command::Arm, argument != 0 ? 1 : 0, now_us);
        break;
    case FrameType::EstopRequest:
        request(core::Command::EmergencyStop, argument != 0 ? 1 : 0, now_us);
        break;
    case FrameType::BringupRun:
        request(core::Command::Bringup, argument, now_us);
        break;
    case FrameType::RebootRequest:
        request(core::Command::Reboot, argument, now_us);
        break;
    default:
        break;
    }
}

void ControlHandler::request(core::Command command, std::uint8_t argument, std::uint64_t now_us)
{
    Pending* slot = nullptr;
    for (Pending& pending : pending_) {
        if (!pending.used) {
            slot = &pending;
            break;
        }
    }
    if (slot == nullptr) {
        return;  // Too many awaiting an answer: the host repeats it.
    }
    ++sequence_;
    *slot = {.sequence = sequence_, .argument = argument, .at_us = now_us, .used = true};
    commands_.publish(msg::VehicleCommand{.timestamp = now_us,
                                          .source = access_.source,
                                          .sequence = sequence_,
                                          .command = static_cast<std::uint8_t>(command),
                                          .argument = argument});
}

void ControlHandler::source_lost(std::uint64_t now_us)
{
    if (!access_.enabled) {
        return;
    }
    ++sequence_;
    commands_.publish(msg::VehicleCommand{.timestamp = now_us,
                                          .source = access_.source,
                                          .sequence = sequence_,
                                          .command = static_cast<std::uint8_t>(core::Command::SourceLost),
                                          .argument = 0});
}

void ControlHandler::send_answers(std::uint64_t now_us, FrameSender& out)
{
    msg::VehicleCommandAck ack{};
    while (acks_.copy(ack)) {
        if (ack.source != access_.source) {
            continue;
        }
        for (Pending& pending : pending_) {
            if (pending.used && pending.sequence == ack.sequence) {
                reply(pending, ack, out);
                pending.used = false;
            }
        }
    }
    for (Pending& pending : pending_) {
        if (pending.used && now_us - pending.at_us > answer_timeout_us) {
            pending.used = false;
        }
    }
}

void ControlHandler::reply(const Pending& pending, const msg::VehicleCommandAck& ack, FrameSender& out)
{
    std::array<std::uint8_t, 5> body{};
    BodyWriter writer(body);
    const bool ok = ack.result == 0;
    switch (static_cast<core::Command>(ack.command)) {
    case core::Command::Mode:
        if (!ok) {
            writer.u8(ack.result);
        }
        writer.u8(ack.mode);
        out.send(ok ? FrameType::ModeReply : FrameType::ModeReject, writer.written());
        break;
    case core::Command::Arm:
        if (!ok) {
            writer.u8(ack.result);
            out.send(FrameType::ArmingReject, writer.written());
            break;
        }
        writer.u8(ack.armed ? 1 : 0);
        writer.u32(ack.skipped_checks);
        out.send(FrameType::ArmingReply, writer.written());
        break;
    case core::Command::EmergencyStop:
        writer.u8(ack.emergency_stop ? 1 : 0);
        out.send(FrameType::EstopReply, writer.written());
        break;
    case core::Command::Bringup:
        if (!ok) {  // Accepted silently: the result follows when the case ends.
            writer.u8(ack.result);
            out.send(FrameType::BringupReject, writer.written());
        }
        break;
    case core::Command::Reboot:
        // The reply echoes the action; the reject gives the reason. The board
        // resets after the reply.
        writer.u8(ok ? pending.argument : ack.result);
        out.send(ok ? FrameType::RebootReply : FrameType::RebootReject, writer.written());
        break;
    default:
        break;
    }
}

bool ControlHandler::send_bringup_result(const msg::BringupStatus& status, FrameSender& out)
{
    std::array<std::uint8_t, 3 + 3 * 4> body{};
    BodyWriter writer(body);
    writer.u8(status.bringup_case);
    writer.u8(status.state);
    writer.u8(status.outputs);
    for (std::size_t i = 0; i < status.outputs && i < status.result.size(); ++i) {
        writer.u8(status.result[i]);
        writer.u8(status.encoder[i]);
        writer.u8(static_cast<std::uint8_t>(status.sign[i]));
    }
    return writer.ok() && out.send(FrameType::BringupResult, writer.written());
}

}  // namespace vk::link
