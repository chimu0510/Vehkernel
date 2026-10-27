// SPDX-License-Identifier: GPL-3.0-or-later
// The commander as a work item on wq:ctrl (ADR-0005, ADR-0008).
//
// Each control cycle it first takes the requests every source published on
// vehicle_command since the last cycle, in order, and answers each on
// vehicle_command_ack with its result and the state after it. Then it
// samples the board's inputs, runs the commander's state machine with the
// COM_* parameters, and publishes arming_state on every change and
// vehicle_status on every change and five times a second. The board counts
// as verified (ADR-0010) once the boot check of the revision-ID pins has
// passed and the IMU has answered with its first sample.
//
// It follows the setpoints on actuator_setpoint and body_velocity_setpoint
// that setpoint arbitration accepts (core::setpoint_accepted, while armed
// and no bring-up case runs), and counts the others on vehicle_status: the
// one accepted last decides whether the targets are idle, and either keeps
// the setpoint timeout from running out. A bring-up request that passes its
// checks goes out on bringup_command; the commander follows the case on
// bringup_status. It refuses arming while local_menu shows the local menu
// open (ADR-0022). From manual_control_input it judges whether the remote
// control is connected (core::rc_connected, RC_TIMEOUT) with its sticks
// centred (MAN_DEADZONE), for MANUAL's arming checks and failsafe.
//
// A reboot accepted while disarmed (ADR-0023) happens reboot_delay_us
// later, from a cycle, so the source's reply leaves first; arming is refused
// meanwhile. vehicle_status also carries the reset cause the platform read
// at boot.
#pragma once

#include <vk/commander/commander.hpp>
#include <vk/core/bus.hpp>
#include <vk/core/fault.hpp>
#include <vk/core/vehicle_control.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/hal/reset.hpp>
#include <vk/msg/actuator_setpoint.hpp>
#include <vk/msg/arming_state.hpp>
#include <vk/msg/body_velocity_setpoint.hpp>
#include <vk/msg/bringup_command.hpp>
#include <vk/msg/bringup_status.hpp>
#include <vk/msg/estimator_restart.hpp>
#include <vk/msg/imu_sample.hpp>
#include <vk/msg/local_menu.hpp>
#include <vk/msg/manual_control_input.hpp>
#include <vk/msg/vehicle_command.hpp>
#include <vk/msg/vehicle_command_ack.hpp>
#include <vk/msg/vehicle_status.hpp>

#include <cstdint>
#include <optional>

namespace vk::commander {

class CommanderTask final : public core::WorkItem {
public:
    static constexpr std::uint32_t status_interval_us = 200'000;
    static constexpr std::uint64_t reboot_delay_us = 100'000;  // a constant

    struct Config {
        std::uint32_t interval_us = 5'000;   // the control cycle
        bool revision_ok = true;             // the revision-ID pins read the board's revision at boot
        bool (*actuator_power)() = nullptr;  // reads the power sense; nullptr if the board has none
        bool actuators_started = false;      // actuator outputs and encoders are running
        bool bringup = false;                // the motor bring-up case is built in
        std::uint32_t (*output_watchdog_trips)() = nullptr;  // the platform's count; nullptr without one
        hal::Reset reset = nullptr;          // the platform's reset; nullptr if it cannot
        core::ResetReport reset_report{};    // what caused the last reset (ADR-0005)
    };

    CommanderTask(const Config& config, core::WorkQueue& queue);

    // Starts the periodic runs.
    void start();

    // From the commander's own work queue, or in tests.
    [[nodiscard]] const State& state() const { return commander_.state(); }
    [[nodiscard]] bool armed() const { return commander_.state().armed; }
    [[nodiscard]] std::uint32_t setpoints_rejected() const { return setpoints_rejected_; }

protected:
    void run() override;

private:
    void handle(std::uint64_t now_us, const msg::VehicleCommand& command);
    [[nodiscard]] core::RebootResult request_reboot(std::uint64_t now_us, std::uint8_t action);
    [[nodiscard]] core::RestartEstimatorsResult request_restart_estimators(std::uint64_t now_us);
    void follow_setpoints();
    void publish(std::uint64_t now_us, const State& state);

    Config config_;
    Commander commander_;
    core::Subscription<msg::VehicleCommand> commands_{0};
    core::Publication<msg::VehicleCommandAck> ack_publication_;
    core::Subscription<msg::ImuSample> imu_{0};
    core::Subscription<msg::ActuatorSetpoint> setpoint_{0};
    msg::ActuatorSetpoint last_setpoint_{};
    core::Subscription<msg::BodyVelocitySetpoint> body_setpoint_{0};
    msg::BodyVelocitySetpoint last_body_setpoint_{};
    std::uint32_t setpoints_rejected_ = 0;
    core::Subscription<msg::BringupStatus> bringup_status_{0};
    core::Subscription<msg::LocalMenu> local_menu_{0};
    core::Subscription<msg::ManualControlInput> manual_input_{0};
    msg::ManualControlInput manual_input_message_{};
    bool local_menu_open_ = false;
    core::Publication<msg::BringupCommand> bringup_publication_;
    bool bringup_running_ = false;
    core::Publication<msg::EstimatorRestart> estimator_restart_publication_;
    core::Publication<msg::ArmingState> arming_publication_;
    core::Publication<msg::VehicleStatus> status_publication_;
    bool imu_answered_ = false;
    bool published_ = false;
    State last_state_{};
    std::uint64_t next_status_us_ = 0;
    struct PendingReboot {
        hal::BootTarget target;
        std::uint64_t at_us;
    };
    std::optional<PendingReboot> reboot_;
};

}  // namespace vk::commander
