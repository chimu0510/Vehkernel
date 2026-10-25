// SPDX-License-Identifier: GPL-3.0-or-later
// The control chain as a work item on wq:ctrl at the base rate (ADR-0008,
// ADR-0013), after the attitude estimator and before actuator control.
//
// Each run takes the newest body velocity setpoint, which only setpoint
// arbitration publishes, through the control chain with MOT_MAX_SPD, and
// publishes the wheel speed targets on actuator_demand, stamped with the
// time that setpoint was accepted (0 and every target NaN before the first
// one). Whether the demand drives anything is actuator control's to decide:
// only while armed, from a setpoint accepted since arming and since the
// emergency stop was released, and only when it is newer than the last
// direct-actuator setpoint (ADR-0005).
#pragma once

#include <vk/control/control_chain.hpp>
#include <vk/core/bus.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/msg/actuator_demand.hpp>
#include <vk/msg/body_velocity_setpoint.hpp>

#include <cstdint>

namespace vk::control {

class ControlChainTask final : public core::WorkItem {
public:
    struct Config {
        std::uint32_t interval_us = 5'000;     // the control cycle
        const VehicleModel* model = nullptr;   // the one selected at boot; nullptr for none
    };

    ControlChainTask(const Config& config, core::WorkQueue& queue);

    // Starts the periodic runs.
    void start();

protected:
    void run() override;

private:
    Config config_;
    ControlChain chain_;
    core::Subscription<msg::BodyVelocitySetpoint> setpoint_{0};
    msg::BodyVelocitySetpoint setpoint_message_{};
    core::Publication<msg::ActuatorDemand> demand_publication_;
};

}  // namespace vk::control
