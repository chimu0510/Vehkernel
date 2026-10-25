// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/control/control_chain_task.hpp>

#include <vk/params/params.hpp>

#include <limits>
#include <span>

namespace vk::control {

ControlChainTask::ControlChainTask(const Config& config, core::WorkQueue& queue)
    : WorkItem("control_chain", queue), config_(config), chain_(config.model)
{
    const float none = std::numeric_limits<float>::quiet_NaN();
    setpoint_message_.vx = none;
    setpoint_message_.vy = none;
    setpoint_message_.yaw_rate = none;
}

void ControlChainTask::start()
{
    schedule_periodic(config_.interval_us);
}

void ControlChainTask::run()
{
    while (setpoint_.copy(setpoint_message_)) {
        // Only the newest counts.
    }
    msg::ActuatorDemand demand{};
    demand.timestamp = queue().now_us();
    demand.timestamp_setpoint = setpoint_message_.timestamp;
    const Twist setpoint{.vx = setpoint_message_.vx, .vy = setpoint_message_.vy,
                         .yaw_rate = setpoint_message_.yaw_rate};
    static_cast<void>(chain_.step(setpoint, params::mot.max_spd, std::span(demand.wheel_speed)));
    demand_publication_.publish(demand);
}

}  // namespace vk::control
