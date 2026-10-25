// SPDX-License-Identifier: GPL-3.0-or-later
// The vehicle control frames of a Link session (ADR-0002, ADR-0005). Mode,
// arming, emergency stop, bring-up and reboot requests go to the commander on
// vehicle_command, with the Link as their source, and are answered once
// vehicle_command_ack brings the commander's answer, a control cycle later:
// mode_request with mode_reply or mode_reject, arming_request with
// arming_reply, which carries the arming checks skipped, or arming_reject,
// estop_request with estop_reply, and reboot_request with reboot_reply or
// reboot_reject. bringup_run is refused with bringup_reject, or accepted
// silently: its bringup_result follows when the case ends. A request the
// commander does not answer within answer_timeout_us, or one beyond
// max_pending awaiting an answer, goes unanswered and the host repeats it.
// Without control access the frames go unanswered and nothing is published.
#pragma once

#include <vk/core/bus.hpp>
#include <vk/core/vehicle_control.hpp>
#include <vk/link/frame.hpp>
#include <vk/link/frame_handler.hpp>
#include <vk/msg/bringup_status.hpp>
#include <vk/msg/vehicle_command.hpp>
#include <vk/msg/vehicle_command_ack.hpp>

#include <array>
#include <cstddef>
#include <cstdint>

namespace vk::link {

// Whether a session may send requests to the commander, and as which source.
struct ControlAccess {
    bool enabled = false;
    core::ControlSource source = 0;  // this Link, as the source of its requests
};

class ControlHandler {
public:
    static constexpr std::size_t max_pending = 4;
    static constexpr std::uint64_t answer_timeout_us = 200'000;

    // Skips the acknowledgements already queued: answers to requests it never sent.
    explicit ControlHandler(ControlAccess access);

    // Publishes the request of a mode_request, arming_request, estop_request,
    // bringup_run or reboot_request frame received at `now_us`.
    void handle(FrameType type, BodyReader body, std::uint64_t now_us);

    // Sends the reply to each request the commander has answered, and
    // forgets those it has not answered within answer_timeout_us.
    void send_answers(std::uint64_t now_us, FrameSender& out);

    // Tells the commander the peer fell silent, which aborts a bring-up case it started.
    void source_lost(std::uint64_t now_us);

    // Sends the result of a bring-up case this session started.
    bool send_bringup_result(const msg::BringupStatus& status, FrameSender& out);

private:
    struct Pending {
        std::uint16_t sequence = 0;
        std::uint8_t argument = 0;
        std::uint64_t at_us = 0;
        bool used = false;
    };

    void request(core::Command command, std::uint8_t argument, std::uint64_t now_us);
    void reply(const Pending& pending, const msg::VehicleCommandAck& ack, FrameSender& out);

    ControlAccess access_;
    core::Publication<msg::VehicleCommand> commands_;
    core::Subscription<msg::VehicleCommandAck> acks_{0};
    std::array<Pending, max_pending> pending_{};
    std::uint16_t sequence_ = 0;
};

}  // namespace vk::link
