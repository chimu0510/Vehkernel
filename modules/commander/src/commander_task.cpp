// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/commander/commander_task.hpp>

#include <vk/params/params.hpp>

#include <algorithm>
#include <cmath>
#include <optional>

namespace vk::commander {

CommanderTask::CommanderTask(const Config& config, core::WorkQueue& queue)
    : WorkItem("commander", queue), config_(config)
{
    // Requests already queued were sent to no commander that exists.
    msg::VehicleCommand stale{};
    while (commands_.copy(stale)) {
    }
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
    msg::VehicleCommand command{};
    while (commands_.copy(command)) {
        handle(now, command);
    }
    follow_setpoints();
    msg::BringupStatus bringup{};
    while (bringup_status_.copy(bringup)) {
        bringup_running_ = bringup.state == 1;  // running
    }
    msg::LocalMenu menu{};
    while (local_menu_.copy(menu)) {
        local_menu_open_ = menu.state != 0;  // list or edit
    }
    // The setpoint level accepted last decides whether the targets are idle.
    const auto unset = [](float target) { return std::isnan(target) || target == 0.0F; };
    const bool body = last_body_setpoint_.timestamp > last_setpoint_.timestamp;
    const bool idle = body ? unset(last_body_setpoint_.vx) && unset(last_body_setpoint_.vy) &&
                                 unset(last_body_setpoint_.yaw_rate)
                           : std::ranges::all_of(last_setpoint_.wheel_speed, unset);
    const Inputs inputs{
        .board_verified = config_.revision_ok && imu_answered_,
        .actuator_power = config_.actuator_power == nullptr ? std::nullopt
                                                             : std::optional<bool>(config_.actuator_power()),
        .actuators_started = config_.actuators_started,
        .setpoints_idle = idle,
        .bringup_running = bringup_running_,
        .bringup_available = config_.bringup,
        .local_menu_open = local_menu_open_,
        .last_setpoint_us = std::max(last_setpoint_.timestamp, last_body_setpoint_.timestamp),
        .output_watchdog_trips = config_.output_watchdog_trips == nullptr ? 0 : config_.output_watchdog_trips(),
        .reset_report = config_.reset_report,
    };
    const Settings settings{.arm_check = static_cast<std::uint32_t>(params::com.arm_check),
                            .setpoint_timeout_s = params::com.sp_timeout,
                            .disarm_idle_s = params::com.disarm_idle};
    commander_.update(now, inputs, settings);
    const State& state = commander_.state();
    publish(now, state);
    if (reboot_ && now >= reboot_->at_us && !state.armed) {
        config_.reset(reboot_->target);  // Does not return.
    }
}

// Setpoint arbitration (ADR-0005): only the setpoints the commander accepts
// keep the timeout from running out or count as idle; the others are counted.
void CommanderTask::follow_setpoints()
{
    msg::ActuatorSetpoint direct{};
    while (setpoint_.copy(direct)) {
        if (commander_.accepts_setpoint(direct.source, core::SetpointLevel::DirectActuator)) {
            last_setpoint_ = direct;
        } else {
            ++setpoints_rejected_;
        }
    }
    msg::BodyVelocitySetpoint body{};
    while (body_setpoint_.copy(body)) {
        if (commander_.accepts_setpoint(body.source, core::SetpointLevel::BodyVelocity)) {
            last_body_setpoint_ = body;
        } else {
            ++setpoints_rejected_;
        }
    }
}

void CommanderTask::handle(std::uint64_t now_us, const msg::VehicleCommand& command)
{
    msg::VehicleCommandAck ack{};
    ack.source = command.source;
    ack.sequence = command.sequence;
    ack.command = command.command;
    switch (static_cast<core::Command>(command.command)) {
    case core::Command::Mode: {
        const bool known = core::known_mode(command.argument);
        ack.result = static_cast<std::uint8_t>(
            known ? commander_.request_mode(now_us, command.source, static_cast<core::Mode>(command.argument))
                  : core::ModeResult::UnknownMode);
        break;
    }
    case core::Command::Arm: {
        const bool arm = command.argument != 0;
        const core::ArmingResult result = arm && reboot_ ? core::ArmingResult::Rebooting
                                                         : commander_.request_arming(now_us, command.source, arm);
        ack.result = static_cast<std::uint8_t>(result);
        break;
    }
    case core::Command::EmergencyStop:
        static_cast<void>(commander_.request_emergency_stop(now_us, command.argument != 0));
        break;
    case core::Command::Bringup: {
        const core::BringupStart result = commander_.request_bringup(command.source, command.argument);
        if (result == core::BringupStart::Ok) {
            bringup_publication_.publish(msg::BringupCommand{
                .timestamp = now_us, .bringup_case = command.argument, .source = command.source});
        }
        ack.result = static_cast<std::uint8_t>(result);
        break;
    }
    case core::Command::Reboot:
        ack.result = static_cast<std::uint8_t>(request_reboot(now_us, command.argument));
        break;
    case core::Command::SourceLost:
        commander_.source_lost(command.source);
        break;
    default:
        return;  // Not a command this commander knows: no answer.
    }
    const State& state = commander_.state();
    ack.timestamp = now_us;
    ack.mode = static_cast<std::uint8_t>(state.mode);
    ack.armed = state.armed;
    ack.emergency_stop = state.emergency_stop;
    ack.skipped_checks = state.skipped_checks;
    ack_publication_.publish(ack);
}

core::RebootResult CommanderTask::request_reboot(std::uint64_t now_us, std::uint8_t action)
{
    const bool known = action == static_cast<std::uint8_t>(core::RebootAction::Reboot) ||
                       action == static_cast<std::uint8_t>(core::RebootAction::Bootloader);
    if (!known) {
        return core::RebootResult::UnknownAction;
    }
    if (config_.reset == nullptr) {
        return core::RebootResult::NotAvailable;
    }
    if (commander_.state().armed) {
        return core::RebootResult::Armed;
    }
    const hal::BootTarget target = action == static_cast<std::uint8_t>(core::RebootAction::Bootloader)
                                       ? hal::BootTarget::Bootloader
                                       : hal::BootTarget::Firmware;
    // A repeated request keeps the first deadline, so retries cannot postpone it.
    if (!reboot_) {
        reboot_ = PendingReboot{.target = target, .at_us = now_us + reboot_delay_us};
    } else {
        reboot_->target = target;
    }
    return core::RebootResult::Ok;
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
        .setpoints_rejected = setpoints_rejected_,
    });
}

}  // namespace vk::commander
