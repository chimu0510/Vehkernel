// SPDX-License-Identifier: GPL-3.0-or-later
// The control chain as a work item on wq:ctrl at the base rate (ADR-0008,
// ADR-0013), after the attitude estimator and before actuator control.
//
// Each run takes the newest body velocity setpoint, which only setpoint
// arbitration publishes, through the control chain with the RATE_YAW_*
// parameters, the yaw rate of vehicle_attitude and MOT_MAX_SPD, and
// publishes the wheel speed targets on actuator_demand, stamped with the
// time that setpoint was accepted (0 and every target NaN before the first
// one), and what allocation made of the demand on allocation_status.
//
// Whether the demand drives anything is actuator control's to decide (only
// while armed, from a setpoint accepted since arming and since the emergency
// stop was released, and newer than the last direct-actuator setpoint,
// ADR-0005); the loops close under the same conditions, judged here the same
// way, so their integrators reset on disarm and the emergency stop and never
// run while another level drives. An attitude older than attitude_timeout_us
// counts as no yaw rate measurement.
#pragma once

#include <vk/control/control_chain.hpp>
#include <vk/core/bus.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/msg/actuator_demand.hpp>
#include <vk/msg/actuator_setpoint.hpp>
#include <vk/msg/allocation_status.hpp>
#include <vk/msg/arming_state.hpp>
#include <vk/msg/body_velocity_setpoint.hpp>
#include <vk/msg/vehicle_attitude.hpp>
#include <vk/msg/vehicle_status.hpp>

#include <cstdint>

namespace vk::control {

class ControlChainTask final : public core::WorkItem {
public:
    static constexpr std::uint64_t attitude_timeout_us = 50'000;  // ten IMU samples at 200 Hz

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
    core::Subscription<msg::ActuatorSetpoint> direct_{0};
    core::Subscription<msg::VehicleAttitude> attitude_{0};
    core::Subscription<msg::ArmingState> arming_{0};
    core::Subscription<msg::VehicleStatus> status_{0};
    msg::BodyVelocitySetpoint setpoint_message_{};
    msg::ActuatorSetpoint direct_message_{};
    msg::VehicleAttitude attitude_message_{};
    core::Publication<msg::ActuatorDemand> demand_publication_;
    core::Publication<msg::AllocationStatus> allocation_publication_;
    bool armed_ = false;
    bool emergency_stop_ = false;
    std::uint64_t setpoints_from_us_ = 0;  // setpoints accepted before this do not count
    std::uint64_t last_run_us_ = 0;
};

}  // namespace vk::control
