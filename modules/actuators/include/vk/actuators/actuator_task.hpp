// Actuator control as a work item on wq:ctrl at the base rate (ADR-0008,
// ADR-0013).
//
// Each run reads the encoders, steps the speed loops with the MOT_*
// parameters, drives each output's PWM and direction lines, and publishes
// wheel_encoders and actuator_outputs. Targets come from actuator_setpoint,
// which only setpoint arbitration publishes, and count only when accepted
// since the vehicle last armed and since the emergency stop was last
// released, so a setpoint left from before a disarm or an emergency stop
// never drives anything: output resumes from the next setpoint. Outputs
// drive only while armed and the emergency stop is released (ADR-0005);
// otherwise both direction lines are low and the motors coast.
//
// With the bring-up case built in, a bringup_command from the commander
// starts the motor-to-encoder case (ADR-0018), which then drives the outputs
// open loop instead of the speed loops and ignores setpoints. Disarming or
// the emergency stop aborts it. Its progress and results go out on
// bringup_status.
#pragma once

#include <vk/actuators/actuator_control.hpp>
#include <vk/actuators/bringup_case.hpp>
#include <vk/core/bus.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/hal/actuator.hpp>
#include <vk/msg/actuator_outputs.hpp>
#include <vk/msg/actuator_setpoint.hpp>
#include <vk/msg/arming_state.hpp>
#include <vk/msg/bringup_command.hpp>
#include <vk/msg/bringup_status.hpp>
#include <vk/msg/vehicle_status.hpp>
#include <vk/msg/wheel_encoders.hpp>

#include <cstdint>
#include <span>

namespace vk::actuators {

class ActuatorTask final : public core::WorkItem {
public:
    struct Config {
        std::uint32_t interval_us = 5'000;          // the control cycle
        std::span<const hal::MotorChannel> channels;  // by output number, output 1 first
        bool bringup = false;                         // the motor bring-up case is built in
    };

    ActuatorTask(const Config& config, core::WorkQueue& queue);

    // Stops every output and starts the periodic runs.
    void start();

protected:
    void run() override;

private:
    void drive(const hal::MotorChannel& channel, float duty);
    void run_bringup(std::uint64_t now_us, std::span<const std::uint16_t> counts);
    void publish_bringup(std::uint64_t now_us);

    Config config_;
    ActuatorControl control_;
    core::Subscription<msg::ActuatorSetpoint> setpoint_{0};
    core::Subscription<msg::ArmingState> arming_{0};
    core::Subscription<msg::VehicleStatus> status_{0};
    core::Subscription<msg::BringupCommand> bringup_command_{0};
    core::Publication<msg::BringupStatus> bringup_publication_;
    BringupCase bringup_;
    BringupState published_state_ = BringupState::Idle;
    std::uint16_t bringup_run_ = 0;
    msg::BringupCommand bringup_request_{};
    core::Publication<msg::WheelEncoders> encoders_publication_;
    core::Publication<msg::ActuatorOutputs> outputs_publication_;
    msg::ActuatorSetpoint setpoint_message_{};
    bool armed_ = false;
    std::uint64_t setpoints_from_us_ = 0;  // setpoints accepted before this do not count
    bool emergency_stop_ = false;
    std::uint64_t last_run_us_ = 0;
};

}  // namespace vk::actuators
