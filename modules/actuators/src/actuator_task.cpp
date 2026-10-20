#include <vk/actuators/actuator_task.hpp>

#include <vk/params/params.hpp>

#include <array>
#include <cmath>
#include <limits>

namespace vk::actuators {
namespace {

// The MOT_* parameters as the speed loops take them.
Settings settings_from_parameters()
{
    const auto& mot = params::mot;
    Settings settings;
    settings.outputs = {{
        {.reversed = mot.mot1_rev != 0, .encoder_reversed = mot.mot1_enc_rev != 0, .counts_per_turn = mot.mot1_cpr},
        {.reversed = mot.mot2_rev != 0, .encoder_reversed = mot.mot2_enc_rev != 0, .counts_per_turn = mot.mot2_cpr},
        {.reversed = mot.mot3_rev != 0, .encoder_reversed = mot.mot3_enc_rev != 0, .counts_per_turn = mot.mot3_cpr},
        {.reversed = mot.mot4_rev != 0, .encoder_reversed = mot.mot4_enc_rev != 0, .counts_per_turn = mot.mot4_cpr},
    }};
    settings.loop = {.p = mot.spd_p, .i = mot.spd_i, .ff = mot.spd_ff, .max_speed = mot.max_spd,
                     .out_max = mot.out_max};
    return settings;
}

}  // namespace

ActuatorTask::ActuatorTask(const Config& config, core::WorkQueue& queue)
    : WorkItem("actuators", queue), config_(config), control_(config.channels.size())
{
    setpoint_message_.wheel_speed.fill(std::numeric_limits<float>::quiet_NaN());
}

void ActuatorTask::start()
{
    for (const hal::MotorChannel& channel : config_.channels) {
        drive(channel, 0.0F);
    }
    schedule_periodic(config_.interval_us);
}

void ActuatorTask::run()
{
    const std::uint64_t now = queue().now_us();
    const float dt = last_run_us_ == 0 ? static_cast<float>(config_.interval_us) * 1e-6F
                                       : static_cast<float>(now - last_run_us_) * 1e-6F;
    last_run_us_ = now;

    msg::ArmingState arming{};
    while (arming_.copy(arming)) {
        if (arming.armed && !armed_) {
            setpoints_from_us_ = arming.timestamp;
        }
        armed_ = arming.armed;
    }
    msg::VehicleStatus status{};
    while (status_.copy(status)) {
        if (emergency_stop_ && !status.emergency_stop) {
            setpoints_from_us_ = status.timestamp;
        }
        emergency_stop_ = status.emergency_stop;
    }
    while (setpoint_.copy(setpoint_message_)) {
        // Only the newest counts.
    }

    std::array<float, max_outputs> targets{};
    targets.fill(std::numeric_limits<float>::quiet_NaN());
    if (armed_ && setpoint_message_.timestamp >= setpoints_from_us_) {
        targets = setpoint_message_.wheel_speed;
    }
    std::array<std::uint16_t, max_outputs> counts{};
    for (std::size_t i = 0; i < control_.size(); ++i) {
        counts[i] = config_.channels[i].encoder->count();
    }
    const auto read = std::span(counts).first(control_.size());
    const Settings settings = settings_from_parameters();
    run_bringup(now, read);
    const bool bringup_running = bringup_.state() == BringupState::Running;
    if (bringup_running) {
        // Setpoints are ignored while the case drives the outputs open loop.
        targets.fill(std::numeric_limits<float>::quiet_NaN());
    }
    control_.step(dt, read, targets, armed_ && !emergency_stop_ && !bringup_running, settings);
    std::array<float, max_outputs> duties{};
    for (std::size_t i = 0; i < control_.size(); ++i) {
        duties[i] = control_.output(i).duty;
    }
    if (bringup_running) {
        duties = bringup_.step(now, read, settings);
    }
    publish_bringup(now);

    msg::WheelEncoders encoders{};
    msg::ActuatorOutputs outputs{};
    encoders.timestamp = now;
    outputs.timestamp = now;
    outputs.target.fill(std::numeric_limits<float>::quiet_NaN());
    for (std::size_t i = 0; i < control_.size(); ++i) {
        const Output& out = control_.output(i);
        drive(config_.channels[i], duties[i]);
        encoders.angle[i] = static_cast<float>(out.angle);
        encoders.speed[i] = out.speed;
        encoders.valid = static_cast<std::uint8_t>(encoders.valid | (1U << i));
        outputs.output[i] = duties[i];
        outputs.target[i] = out.target;
    }
    encoders_publication_.publish(encoders);
    outputs_publication_.publish(outputs);
}

void ActuatorTask::run_bringup(std::uint64_t now_us, std::span<const std::uint16_t> counts)
{
    msg::BringupCommand command{};
    while (bringup_command_.copy(command)) {
        if (config_.bringup && armed_ && !emergency_stop_ && command.timestamp >= setpoints_from_us_ &&
            bringup_.state() != BringupState::Running) {
            bringup_request_ = command;
            ++bringup_run_;
            bringup_.start(now_us, counts.size(), counts);
        }
    }
    if (bringup_.state() == BringupState::Running && (!armed_ || emergency_stop_)) {
        bringup_.abort();
    }
}

void ActuatorTask::publish_bringup(std::uint64_t now_us)
{
    if (bringup_.state() == published_state_) {
        return;
    }
    published_state_ = bringup_.state();
    msg::BringupStatus status{};
    status.timestamp = now_us;
    status.run = bringup_run_;
    status.bringup_case = bringup_request_.bringup_case;
    status.source = bringup_request_.source;
    status.state = static_cast<std::uint8_t>(published_state_);
    status.outputs = static_cast<std::uint8_t>(bringup_.outputs());
    for (std::size_t i = 0; i < bringup_.outputs(); ++i) {
        const OutputResult& result = bringup_.result(i);
        status.result[i] = static_cast<std::uint8_t>(result.result);
        status.encoder[i] = result.encoder;
        status.sign[i] = result.sign;
    }
    bringup_publication_.publish(status);
}

void ActuatorTask::drive(const hal::MotorChannel& channel, float duty)
{
    // Direction first, so the duty never drives the old direction.
    channel.direction->set(duty > 0.0F, duty < 0.0F);
    channel.pwm->set_duty(std::fabs(duty));
}

}  // namespace vk::actuators
