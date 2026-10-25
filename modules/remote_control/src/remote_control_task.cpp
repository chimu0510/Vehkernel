// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/remote_control/remote_control_task.hpp>

#include <vk/core/manual_control.hpp>
#include <vk/params/params.hpp>

namespace vk::remote_control {

RemoteControlTask::RemoteControlTask(const Config& config, core::WorkQueue& queue)
    : WorkItem("remote_control", queue), config_(config)
{
}

void RemoteControlTask::start()
{
    schedule_periodic(config_.interval_us);
}

void RemoteControlTask::run()
{
    const std::uint64_t now = queue().now_us();
    while (input_.copy(input_message_)) {
        // Only the newest counts.
    }
    msg::VehicleStatus status{};
    while (status_.copy(status)) {
        mode_ = static_cast<core::Mode>(status.mode);
        in_control_ = status.source;
    }
    const auto timeout_us = static_cast<std::uint64_t>(params::rc.timeout * 1e6F);
    const Input input{.connected = core::rc_connected(input_message_, now, timeout_us),
                      .sticks = input_message_.sticks,
                      .buttons = input_message_.buttons};
    const Settings settings{.vx_max = params::man.vx_max, .yaw_max = params::man.yaw_max,
                            .deadzone = params::man.deadzone};
    const Result result = remote_control_.step(now, input, mode_, in_control_, settings);
    for (std::size_t i = 0; i < result.request_count; ++i) {
        ++sequence_;
        command_publication_.publish(msg::VehicleCommand{.timestamp = now,
                                                         .source = core::remote_control,
                                                         .sequence = sequence_,
                                                         .command = static_cast<std::uint8_t>(result.requests[i].command),
                                                         .argument = result.requests[i].argument});
    }
    if (result.setpoint) {
        setpoint_publication_.publish(msg::BodyVelocitySetpoint{.timestamp = now,
                                                                .source = core::remote_control,
                                                                .vx = result.setpoint->vx,
                                                                .vy = result.setpoint->vy,
                                                                .yaw_rate = result.setpoint->yaw_rate});
    }
}

}  // namespace vk::remote_control
