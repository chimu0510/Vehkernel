// SPDX-License-Identifier: GPL-3.0-or-later
// The commander as a work item on wq:ctrl (ADR-0005, ADR-0008).
//
// Each control cycle it samples the board's inputs, runs the commander's
// state machine with the COM_* parameters, and publishes arming_state on
// every change and vehicle_status on every change and five times a second.
// The board counts as verified (ADR-0010) once the boot check of the
// revision-ID pins has passed and the IMU has answered with its first sample.
// It follows the setpoints that arbitration accepted on actuator_setpoint,
// starts bring-up cases on bringup_command and follows them on
// bringup_status. It refuses arming while local_menu shows the local menu
// open (ADR-0022).
//
// It is also the vehicle control service the Links call from their own work
// queue; requests and cycles take turns in short critical sections. A reboot
// accepted while disarmed (ADR-0023) happens reboot_delay_us later, from a
// cycle, so the Link's reply leaves first; arming is refused meanwhile.
#pragma once

#include <vk/commander/commander.hpp>
#include <vk/core/bus.hpp>
#include <vk/core/vehicle_control.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/hal/reset.hpp>
#include <vk/msg/actuator_setpoint.hpp>
#include <vk/msg/arming_state.hpp>
#include <vk/msg/bringup_command.hpp>
#include <vk/msg/bringup_status.hpp>
#include <vk/msg/imu_sample.hpp>
#include <vk/msg/local_menu.hpp>
#include <vk/msg/vehicle_status.hpp>

#include <cstdint>
#include <optional>

namespace vk::commander {

class CommanderTask final : public core::WorkItem, public core::VehicleControl {
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
    };

    CommanderTask(const Config& config, core::WorkQueue& queue);

    // Starts the periodic runs.
    void start();

    core::ModeResult request_mode(core::ControlSource source, core::Mode mode) override;
    core::ArmingResult request_arming(core::ControlSource source, bool arm) override;
    bool request_emergency_stop(bool engage) override;
    [[nodiscard]] core::Mode mode() const override;
    [[nodiscard]] bool armed() const override;
    [[nodiscard]] std::uint32_t skipped_checks() const override;
    [[nodiscard]] bool accepts_setpoint(core::ControlSource source) const override;
    core::BringupStart request_bringup(core::ControlSource source, std::uint8_t bringup_case) override;
    void source_lost(core::ControlSource source) override;
    core::RebootResult request_reboot(core::RebootAction action) override;

protected:
    void run() override;

private:
    void publish(std::uint64_t now_us, const State& state);

    Config config_;
    Commander commander_;
    core::Subscription<msg::ImuSample> imu_{0};
    core::Subscription<msg::ActuatorSetpoint> setpoint_{0};
    msg::ActuatorSetpoint last_setpoint_{};
    core::Subscription<msg::BringupStatus> bringup_status_{0};
    core::Subscription<msg::LocalMenu> local_menu_{0};
    bool local_menu_open_ = false;
    core::Publication<msg::BringupCommand> bringup_publication_;
    bool bringup_running_ = false;
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
    std::optional<PendingReboot> reboot_;  // guarded by the critical section
};

}  // namespace vk::commander
