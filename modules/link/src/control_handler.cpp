// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/link/control_handler.hpp>

#include <array>
#include <cstddef>
#include <cstdint>

namespace vk::link {

void ControlHandler::handle(FrameType type, BodyReader body, FrameSender& out)
{
    switch (type) {
    case FrameType::ModeRequest:
        on_mode_request(body, out);
        break;
    case FrameType::ArmingRequest:
        on_arming_request(body, out);
        break;
    case FrameType::EstopRequest:
        on_estop_request(body, out);
        break;
    case FrameType::BringupRun:
        on_bringup_run(body, out);
        break;
    case FrameType::RebootRequest:
        on_reboot_request(body, out);
        break;
    default:
        break;
    }
}

void ControlHandler::on_mode_request(BodyReader body, FrameSender& out)
{
    const std::uint8_t mode = body.u8();
    if (!body.ok() || access_.control == nullptr) {
        return;
    }
    const core::ModeResult result =
        mode > static_cast<std::uint8_t>(core::Mode::Test)
            ? core::ModeResult::UnknownMode
            : access_.control->request_mode(access_.source, static_cast<core::Mode>(mode));
    std::array<std::uint8_t, 2> reply{};
    BodyWriter writer(reply);
    if (result != core::ModeResult::Ok) {
        writer.u8(static_cast<std::uint8_t>(result));
        writer.u8(static_cast<std::uint8_t>(access_.control->mode()));
        out.send(FrameType::ModeReject, writer.written());
        return;
    }
    writer.u8(static_cast<std::uint8_t>(access_.control->mode()));
    out.send(FrameType::ModeReply, writer.written());
}

void ControlHandler::on_arming_request(BodyReader body, FrameSender& out)
{
    const std::uint8_t arm = body.u8();
    if (!body.ok() || access_.control == nullptr) {
        return;
    }
    const core::ArmingResult result = access_.control->request_arming(access_.source, arm != 0);
    std::array<std::uint8_t, 5> reply{};
    BodyWriter writer(reply);
    if (result != core::ArmingResult::Ok) {
        writer.u8(static_cast<std::uint8_t>(result));
        out.send(FrameType::ArmingReject, writer.written());
        return;
    }
    writer.u8(access_.control->armed() ? 1 : 0);
    writer.u32(access_.control->skipped_checks());
    out.send(FrameType::ArmingReply, writer.written());
}

void ControlHandler::on_estop_request(BodyReader body, FrameSender& out)
{
    const std::uint8_t engage = body.u8();
    if (!body.ok() || access_.control == nullptr) {
        return;
    }
    std::array<std::uint8_t, 1> reply{};
    BodyWriter writer(reply);
    writer.u8(access_.control->request_emergency_stop(engage != 0) ? 1 : 0);
    out.send(FrameType::EstopReply, writer.written());
}

void ControlHandler::on_bringup_run(BodyReader body, FrameSender& out)
{
    const std::uint8_t bringup_case = body.u8();
    if (!body.ok() || access_.control == nullptr) {
        return;
    }
    const core::BringupStart result = access_.control->request_bringup(access_.source, bringup_case);
    if (result == core::BringupStart::Ok) {
        return;  // The result follows when the case ends.
    }
    std::array<std::uint8_t, 1> reply{static_cast<std::uint8_t>(result)};
    out.send(FrameType::BringupReject, reply);
}

// reboot_request: the action (core::RebootAction). The reply echoes it; the
// reject gives the reason (core::RebootResult). The board resets after the reply.
void ControlHandler::on_reboot_request(BodyReader body, FrameSender& out)
{
    const std::uint8_t action = body.u8();
    if (!body.ok() || access_.control == nullptr) {
        return;
    }
    const bool known = action == static_cast<std::uint8_t>(core::RebootAction::Reboot) ||
                       action == static_cast<std::uint8_t>(core::RebootAction::Bootloader);
    const core::RebootResult result = known ? access_.control->request_reboot(static_cast<core::RebootAction>(action))
                                            : core::RebootResult::UnknownAction;
    std::array<std::uint8_t, 1> reply{};
    if (result != core::RebootResult::Ok) {
        reply[0] = static_cast<std::uint8_t>(result);
        out.send(FrameType::RebootReject, reply);
        return;
    }
    reply[0] = action;
    out.send(FrameType::RebootReply, reply);
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
