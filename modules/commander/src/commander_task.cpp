// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/commander/commander_task.hpp>

#include <vk/core/critical_section.hpp>
#include <vk/params/params.hpp>

#include <algorithm>
#include <cmath>
#include <optional>

namespace vk::commander {

CommanderTask::CommanderTask(const Config& config, core::WorkQueue& queue)
    : WorkItem("commander", queue), config_(config)
{
}

void CommanderTask::start()
{
    schedule_periodic(config_.interval_us);
}

void CommanderTask::run()
{
    const std::uint64_t now = queue().now_us();
    if (imu_.updated()) {
        imu_answered_ = true;
    }
    while (setpoint_.copy(last_setpoint_)) {
        // Only the newest counts.
    }
    msg::BringupStatus bringup{};
    while (bringup_status_.copy(bringup)) {
        bringup_running_ = bringup.state == 1;  // running
    }
    msg::LocalMenu menu{};
    while (local_menu_.copy(menu)) {
        local_menu_open_ = menu.state != 0;  // list or edit
    }
    const bool idle = std::ranges::all_of(last_setpoint_.wheel_speed,
                                          [](float target) { return std::isnan(target) || target == 0.0F; });
    const Inputs inputs{
        .board_verified = config_.revision_ok && imu_answered_,
        .actuator_power = config_.actuator_power == nullptr ? std::nullopt
                                                             : std::optional<bool>(config_.actuator_power()),
        .actuators_started = config_.actuators_started,
        .setpoints_idle = idle,
        .bringup_running = bringup_running_,
        .bringup_available = config_.bringup,
        .local_menu_open = local_menu_open_,
        .last_setpoint_us = last_setpoint_.timestamp,
        .output_watchdog_trips = config_.output_watchdog_trips == nullptr ? 0 : config_.output_watchdog_trips(),
        .reset_report = config_.reset_report,
    };
    const Settings settings{.arm_check = static_cast<std::uint32_t>(params::com.arm_check),
                            .setpoint_timeout_s = params::com.sp_timeout,
                            .disarm_idle_s = params::com.disarm_idle};
    State state;
    std::optional<PendingReboot> reboot;
    {
        const core::CriticalSection lock;
        commander_.update(now, inputs, settings);
        state = commander_.state();
        reboot = reboot_;
    }
    publish(now, state);
    if (reboot && now >= reboot->at_us && !state.armed) {
        config_.reset(reboot->target);  // Does not return.
    }
}

void CommanderTask::publish(std::uint64_t now_us, const State& state)
{
    if (!published_ || state.armed != last_state_.armed) {
        arming_publication_.publish(msg::ArmingState{.timestamp = now_us, .armed = state.armed});
    }
    if (published_ && state == last_state_ && now_us < next_status_us_) {
        return;
    }
    published_ = true;
    last_state_ = state;
    next_status_us_ = now_us + status_interval_us;
    status_publication_.publish(msg::VehicleStatus{
        .timestamp = now_us,
        .mode = static_cast<std::uint8_t>(state.mode),
        .armed = state.armed,
        .source = state.source,
        .board_verified = state.board_verified,
        .emergency_stop = state.emergency_stop,
        .actuator_power = static_cast<std::uint8_t>(state.actuator_power),
        .actuator_power_raw = state.actuator_power_raw,
        .skipped_checks = state.skipped_checks,
        .arming_refusal = static_cast<std::uint8_t>(state.arming_refusal),
        .disarm_reason = static_cast<std::uint8_t>(state.disarm_reason),
        .output_watchdog_trips = state.output_watchdog_trips,
        .reset_cause = static_cast<std::uint8_t>(state.reset_report.cause),
        .fault = static_cast<std::uint8_t>(state.reset_report.fault),
        .fault_address = state.reset_report.fault_address,
    });
}

core::ModeResult CommanderTask::request_mode(core::ControlSource source, core::Mode mode)
{
    const core::CriticalSection lock;
    return commander_.request_mode(queue().now_us(), source, mode);
}

core::ArmingResult CommanderTask::request_arming(core::ControlSource source, bool arm)
{
    const core::CriticalSection lock;
    if (arm && reboot_) {
        return core::ArmingResult::Rebooting;
    }
    return commander_.request_arming(queue().now_us(), source, arm);
}

core::RebootResult CommanderTask::request_reboot(core::RebootAction action)
{
    if (config_.reset == nullptr) {
        return core::RebootResult::NotAvailable;
    }
    const core::CriticalSection lock;
    if (commander_.state().armed) {
        return core::RebootResult::Armed;
    }
    const hal::BootTarget target =
        action == core::RebootAction::Bootloader ? hal::BootTarget::Bootloader : hal::BootTarget::Firmware;
    // A repeated request keeps the first deadline, so retries cannot postpone it.
    if (!reboot_) {
        reboot_ = PendingReboot{.target = target, .at_us = queue().now_us() + reboot_delay_us};
    } else {
        reboot_->target = target;
    }
    return core::RebootResult::Ok;
}

bool CommanderTask::request_emergency_stop(bool engage)
{
    const core::CriticalSection lock;
    return commander_.request_emergency_stop(queue().now_us(), engage);
}

core::Mode CommanderTask::mode() const
{
    const core::CriticalSection lock;
    return commander_.state().mode;
}

bool CommanderTask::armed() const
{
    const core::CriticalSection lock;
    return commander_.state().armed;
}

core::BringupStart CommanderTask::request_bringup(core::ControlSource source, std::uint8_t bringup_case)
{
    core::BringupStart result{};
    {
        const core::CriticalSection lock;
        result = commander_.request_bringup(source, bringup_case);
    }
    if (result == core::BringupStart::Ok) {
        bringup_publication_.publish(
            msg::BringupCommand{.timestamp = queue().now_us(), .bringup_case = bringup_case, .source = source});
    }
    return result;
}

void CommanderTask::source_lost(core::ControlSource source)
{
    const core::CriticalSection lock;
    commander_.source_lost(source);
}

bool CommanderTask::accepts_setpoint(core::ControlSource source) const
{
    const core::CriticalSection lock;
    return commander_.accepts_setpoint(source);
}

std::uint32_t CommanderTask::skipped_checks() const
{
    const core::CriticalSection lock;
    return commander_.state().skipped_checks;
}

}  // namespace vk::commander
