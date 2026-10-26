// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/control/control_chain_task.hpp>

#include <vk/params/params.hpp>

#include <limits>
#include <optional>
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
    const std::uint64_t now = queue().now_us();
    const float dt = last_run_us_ == 0 ? static_cast<float>(config_.interval_us) * 1e-6F
                                       : static_cast<float>(now - last_run_us_) * 1e-6F;
    last_run_us_ = now;

    msg::ArmingState arming{};
    while (arming_.copy(arming)) {
        if (arming.armed && !armed_) {
            setpoints_from_us_ = arming.timestamp;
        }
        armed_ = arming.armed;
    }
    msg::VehicleStatus status{};
    while (status_.copy(status)) {
        if (emergency_stop_ && !status.emergency_stop) {
            setpoints_from_us_ = status.timestamp;
        }
        emergency_stop_ = status.emergency_stop;
        mode_ = static_cast<core::Mode>(status.mode);
        in_control_ = status.source;
    }
    // The newest of each level that setpoint arbitration accepts counts.
    msg::BodyVelocitySetpoint body{};
    while (setpoint_.copy(body)) {
        if (core::setpoint_accepted(mode_, in_control_, body.source, core::SetpointLevel::BodyVelocity)) {
            setpoint_message_ = body;
        }
    }
    msg::ActuatorSetpoint direct{};
    while (direct_.copy(direct)) {
        if (core::setpoint_accepted(mode_, in_control_, direct.source, core::SetpointLevel::DirectActuator)) {
            direct_message_ = direct;
        }
    }
    while (attitude_.copy(attitude_message_)) {
    }

    // The loops close while this setpoint drives, as actuator control decides it.
    const std::uint64_t accepted = setpoint_message_.timestamp;
    const bool drives = accepted != 0 && accepted >= setpoints_from_us_ && accepted > direct_message_.timestamp;
    const bool closed = armed_ && !emergency_stop_ && drives;
    std::optional<float> yaw_rate;
    if (attitude_message_.timestamp != 0 && now - attitude_message_.timestamp <= attitude_timeout_us) {
        yaw_rate = attitude_message_.angular_velocity[2];
    }
    const RateSettings settings{.p = params::rate.yaw_p, .i = params::rate.yaw_i, .ff = params::rate.yaw_ff};
    const VelocityLimits limits{.acc_max = params::vel.acc_max, .dec_max = params::vel.dec_max};
    const Twist setpoint{.vx = setpoint_message_.vx, .vy = setpoint_message_.vy,
                         .yaw_rate = setpoint_message_.yaw_rate};

    msg::ActuatorDemand demand{};
    demand.timestamp = now;
    demand.timestamp_setpoint = accepted;
    const ChainResult result =
        chain_.step(dt, setpoint, yaw_rate, closed, settings, params::mot.max_spd, std::span(demand.wheel_speed), limits);
    demand_publication_.publish(demand);
    if (closed) {
        msg::AllocationStatus allocation{};
        allocation.timestamp = now;
        allocation.vx = result.demand.vx;
        allocation.vy = result.demand.vy;
        allocation.yaw_rate = result.demand.yaw_rate;
        allocation.scale = result.allocation.scale;
        allocation.limited = result.allocation.limited;
        allocation_publication_.publish(allocation);
    }
}

}  // namespace vk::control
