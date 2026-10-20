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
    control_.step(dt, std::span(counts).first(control_.size()), targets, armed_ && !emergency_stop_,
                  settings_from_parameters());

    msg::WheelEncoders encoders{};
    msg::ActuatorOutputs outputs{};
    encoders.timestamp = now;
    outputs.timestamp = now;
    outputs.target.fill(std::numeric_limits<float>::quiet_NaN());
    for (std::size_t i = 0; i < control_.size(); ++i) {
        const Output& out = control_.output(i);
        drive(config_.channels[i], out.duty);
        encoders.angle[i] = static_cast<float>(out.angle);
        encoders.speed[i] = out.speed;
        encoders.valid = static_cast<std::uint8_t>(encoders.valid | (1U << i));
        outputs.output[i] = out.duty;
        outputs.target[i] = out.target;
    }
    encoders_publication_.publish(encoders);
    outputs_publication_.publish(outputs);
}

void ActuatorTask::drive(const hal::MotorChannel& channel, float duty)
{
    // Direction first, so the duty never drives the old direction.
    channel.direction->set(duty > 0.0F, duty < 0.0F);
    channel.pwm->set_duty(std::fabs(duty));
}

}  // namespace vk::actuators
