// SPDX-License-Identifier: GPL-3.0-or-later
// The vehicle control frames of a Link session (ADR-0002, ADR-0005). Mode,
// arming, emergency stop, bring-up and reboot requests go to the vehicle
// control service, with the Link as their source, and are answered at once:
// mode_request with mode_reply or mode_reject, arming_request with
// arming_reply, which carries the arming checks skipped, or arming_reject,
// estop_request with estop_reply, and reboot_request with reboot_reply or
// reboot_reject. bringup_run is refused at once with bringup_reject, or
// accepted silently: its bringup_result follows when the case ends. Without
// the service they go unanswered.
#pragma once

#include <vk/core/vehicle_control.hpp>
#include <vk/link/frame.hpp>
#include <vk/link/frame_handler.hpp>
#include <vk/msg/bringup_status.hpp>

namespace vk::link {

// The vehicle control a session offers; without it, mode, arming and
// emergency stop requests go unanswered.
struct ControlAccess {
    core::VehicleControl* control = nullptr;
    core::ControlSource source = 0;                     // this Link, as the source of its requests
};

class ControlHandler {
public:
    explicit ControlHandler(ControlAccess access) : access_(access) {}

    // Handles mode_request, arming_request, estop_request, bringup_run or
    // reboot_request.
    void handle(FrameType type, BodyReader body, FrameSender& out);

    // Sends the result of a bring-up case this session started.
    bool send_bringup_result(const msg::BringupStatus& status, FrameSender& out);

private:
    void on_mode_request(BodyReader body, FrameSender& out);
    void on_arming_request(BodyReader body, FrameSender& out);
    void on_estop_request(BodyReader body, FrameSender& out);
    void on_bringup_run(BodyReader body, FrameSender& out);
    void on_reboot_request(BodyReader body, FrameSender& out);

    ControlAccess access_;
};

}  // namespace vk::link
