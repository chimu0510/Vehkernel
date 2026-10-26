// SPDX-License-Identifier: GPL-3.0-or-later
// The remote control as a work item on wq:ctrl at the base rate (spec 0010,
// ADR-0008), before the control chain.
//
// Each run takes the newest manual_control_input, which counts as connected
// while it says so and is no older than RC_TIMEOUT, and the mode and source
// in control from vehicle_status, steps the remote control with the MAN_*
// parameters, publishes its body velocity setpoint with the remote control
// as source, and sends its requests on vehicle_command as that source. It
// ignores the acknowledgements: the operator sees the vehicle respond. The
// commander judges the same input for MANUAL's arming checks and failsafe.
#pragma once

#include <vk/core/bus.hpp>
#include <vk/core/vehicle_control.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/msg/body_velocity_setpoint.hpp>
#include <vk/msg/manual_control_input.hpp>
#include <vk/msg/vehicle_command.hpp>
#include <vk/msg/vehicle_status.hpp>
#include <vk/remote_control/remote_control.hpp>

#include <cstdint>

namespace vk::remote_control {

class RemoteControlTask final : public core::WorkItem {
public:
    struct Config {
        std::uint32_t interval_us = 5'000;  // the control cycle
    };

    RemoteControlTask(const Config& config, core::WorkQueue& queue);

    // Starts the periodic runs.
    void start();

protected:
    void run() override;

private:
    Config config_;
    RemoteControl remote_control_;
    core::Subscription<msg::ManualControlInput> input_{0};
    core::Subscription<msg::VehicleStatus> status_{0};
    msg::ManualControlInput input_message_{};
    core::Mode mode_ = core::Mode::Standby;
    core::ControlSource in_control_ = core::no_source;
    core::Publication<msg::BodyVelocitySetpoint> setpoint_publication_;
    core::Publication<msg::VehicleCommand> command_publication_;
    std::uint16_t sequence_ = 0;
};

}  // namespace vk::remote_control
