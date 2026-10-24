// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/commander/commander.hpp>

#include <algorithm>

namespace vk::commander {
namespace {

constexpr std::uint64_t to_us(float seconds)
{
    return static_cast<std::uint64_t>(seconds * 1e6F);
}

}  // namespace

void Commander::update(std::uint64_t now_us, const Inputs& inputs, const Settings& settings)
{
    inputs_ = inputs;
    settings_ = settings;
    state_.board_verified = inputs.board_verified;
    state_.output_watchdog_trips = inputs.output_watchdog_trips;
    state_.reset_report = inputs.reset_report;
    state_.skipped_checks = ~settings.arm_check & core::arm_checks::all;
    track_power(now_us, inputs.actuator_power);
    if (!state_.armed) {
        return;
    }
    if (state_.actuator_power == ActuatorPower::Unpowered && power_checked()) {
        disarm(core::DisarmReason::ActuatorPowerLost);
        return;
    }
    if (state_.emergency_stop && inputs.bringup_running) {
        disarm(core::DisarmReason::BringupAbort);
        return;
    }
    if (state_.emergency_stop && now_us - engaged_at_us_ >= emergency_stop_disarm_us) {
        disarm(core::DisarmReason::EmergencyStop);
        return;
    }
    const bool setpoint_since_arming = inputs.last_setpoint_us != 0 && inputs.last_setpoint_us >= armed_at_us_;
    if ((setpoint_since_arming && !inputs.setpoints_idle) || inputs.bringup_running) {
        active_at_us_ = now_us;
    }
    if (now_us - active_at_us_ >= to_us(settings.disarm_idle_s)) {
        disarm(core::DisarmReason::Idle);
        return;
    }
    // Neither the emergency stop nor a bring-up case counts against the source in control.
    if (state_.emergency_stop || inputs.bringup_running) {
        timeout_from_us_ = now_us;
    }
    if (setpoint_since_arming &&
        now_us - std::max(inputs.last_setpoint_us, timeout_from_us_) > to_us(settings.setpoint_timeout_s)) {
        disarm(core::DisarmReason::SetpointTimeout);
    }
}

core::ModeResult Commander::request_mode(std::uint64_t /*now_us*/, core::ControlSource source, core::Mode mode)
{
    if (mode != core::Mode::Standby && mode != core::Mode::Test) {
        return core::ModeResult::UnknownMode;
    }
    const core::ControlSource in_control = mode == core::Mode::Test ? source : core::no_source;
    if (state_.armed) {
        // A repeated request changes nothing, so it is not refused.
        const bool unchanged = mode == state_.mode && in_control == state_.source;
        return unchanged ? core::ModeResult::Ok : core::ModeResult::Armed;
    }
    state_.mode = mode;
    state_.source = in_control;
    return core::ModeResult::Ok;
}

core::ArmingResult Commander::request_arming(std::uint64_t now_us, core::ControlSource source, bool arm_request)
{
    if (!arm_request) {
        if (state_.armed) {
            disarm(core::DisarmReason::Request);
        }
        return core::ArmingResult::Ok;
    }
    if (state_.armed) {
        return core::ArmingResult::Ok;
    }
    const core::ArmingResult result = arming_check(now_us, source);
    state_.arming_refusal = result;
    if (result == core::ArmingResult::Ok) {
        arm(now_us);
    }
    return result;
}

bool Commander::request_emergency_stop(std::uint64_t now_us, bool engage)
{
    if (engage && !state_.emergency_stop) {
        engaged_at_us_ = now_us;
    }
    state_.emergency_stop = engage;
    return state_.emergency_stop;
}

core::BringupStart Commander::request_bringup(core::ControlSource source, std::uint8_t bringup_case) const
{
    if (!inputs_.bringup_available) {
        return core::BringupStart::NotAvailable;
    }
    if (bringup_case != static_cast<std::uint8_t>(core::BringupCase::Motors)) {
        return core::BringupStart::UnknownCase;
    }
    if (state_.mode != core::Mode::Test) {
        return core::BringupStart::WrongMode;
    }
    if (!state_.armed) {
        return core::BringupStart::NotArmed;
    }
    if (source != state_.source) {
        return core::BringupStart::NotInControl;
    }
    if (state_.emergency_stop) {
        return core::BringupStart::EmergencyStop;
    }
    if (inputs_.bringup_running) {
        return core::BringupStart::AlreadyRunning;
    }
    return core::BringupStart::Ok;
}

void Commander::source_lost(core::ControlSource source)
{
    if (state_.armed && inputs_.bringup_running && source == state_.source) {
        disarm(core::DisarmReason::BringupAbort);
    }
}

bool Commander::accepts_setpoint(core::ControlSource source) const
{
    return state_.armed && state_.mode == core::Mode::Test && source == state_.source && !inputs_.bringup_running;
}

core::ArmingResult Commander::arming_check(std::uint64_t now_us, core::ControlSource source) const
{
    if (!state_.board_verified) {
        return core::ArmingResult::BoardNotVerified;
    }
    if (state_.emergency_stop) {
        return core::ArmingResult::EmergencyStop;
    }
    if (state_.mode != core::Mode::Test) {
        return core::ArmingResult::WrongMode;
    }
    if (source != state_.source) {
        return core::ArmingResult::NotInControl;
    }
    const bool current = inputs_.last_setpoint_us != 0 &&
                         now_us - inputs_.last_setpoint_us <= to_us(settings_.setpoint_timeout_s);
    if (current && !inputs_.setpoints_idle) {
        return core::ArmingResult::SetpointsActive;
    }
    if (inputs_.bringup_running) {
        return core::ArmingResult::BringupRunning;
    }
    if (power_checked() && (state_.actuator_power == ActuatorPower::Unpowered ||
                            state_.actuator_power == ActuatorPower::Settling)) {
        return core::ArmingResult::ActuatorPowerOff;
    }
    if (!inputs_.actuators_started) {
        return core::ArmingResult::ActuatorsNotStarted;
    }
    if (inputs_.local_menu_open) {
        return core::ArmingResult::LocalMenuOpen;
    }
    if (core::unexpected(state_.reset_report)) {
        return core::ArmingResult::UnexpectedReset;
    }
    return core::ArmingResult::Ok;
}

bool Commander::power_checked() const
{
    return (settings_.arm_check & core::arm_checks::actuator_power) != 0U;
}

void Commander::track_power(std::uint64_t now_us, std::optional<bool> sample)
{
    if (!sample) {
        state_.actuator_power = ActuatorPower::NotAvailable;
        state_.actuator_power_raw = false;
        return;
    }
    state_.actuator_power_raw = *sample;
    if (!*sample) {
        state_.actuator_power = ActuatorPower::Unpowered;
        return;
    }
    if (state_.actuator_power == ActuatorPower::Unpowered || state_.actuator_power == ActuatorPower::NotAvailable) {
        state_.actuator_power = ActuatorPower::Settling;
        power_since_us_ = now_us;
    }
    if (state_.actuator_power == ActuatorPower::Settling && now_us - power_since_us_ >= power_settle_us) {
        state_.actuator_power = ActuatorPower::Powered;
    }
}

void Commander::arm(std::uint64_t now_us)
{
    state_.armed = true;
    armed_at_us_ = now_us;
    active_at_us_ = now_us;
    timeout_from_us_ = now_us;
}

void Commander::disarm(core::DisarmReason reason)
{
    state_.armed = false;
    state_.disarm_reason = reason;
}

}  // namespace vk::commander
