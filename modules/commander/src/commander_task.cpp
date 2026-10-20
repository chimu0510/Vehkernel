#include <vk/commander/commander_task.hpp>

#include <vk/core/critical_section.hpp>
#include <vk/params/params.hpp>

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
    const Inputs inputs{
        .board_verified = config_.revision_ok && imu_answered_,
        .actuator_power = config_.actuator_power == nullptr ? std::nullopt
                                                             : std::optional<bool>(config_.actuator_power()),
        .actuators_started = config_.actuators_started,
    };
    const Settings settings{.arm_check = static_cast<std::uint32_t>(params::com.arm_check),
                            .setpoint_timeout_s = params::com.sp_timeout,
                            .disarm_idle_s = params::com.disarm_idle};
    State state;
    {
        const core::CriticalSection lock;
        commander_.update(now, inputs, settings);
        state = commander_.state();
    }
    publish(now, state);
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
        .output_watchdog_trips = 0,
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
    return commander_.request_arming(queue().now_us(), source, arm);
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

std::uint32_t CommanderTask::skipped_checks() const
{
    const core::CriticalSection lock;
    return commander_.state().skipped_checks;
}

}  // namespace vk::commander
