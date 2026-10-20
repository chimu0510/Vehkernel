#include <vk/actuators/actuator_control.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace vk::actuators {

float slew(float previous, float requested, float rate, float dt_s)
{
    if (rate <= 0.0F || requested == 0.0F) {
        return requested;
    }
    const float step = rate * dt_s;
    return std::clamp(requested, previous - step, previous + step);
}

ActuatorControl::ActuatorControl(std::size_t outputs) : size_(std::min(outputs, max_outputs)) {}

void ActuatorControl::step(float dt_s, std::span<const std::uint16_t> counts, std::span<const float> targets,
                           bool enabled, const Settings& settings, std::span<const float> previous)
{
    const LoopSettings& loop = settings.loop;
    for (std::size_t i = 0; i < size_ && i < counts.size(); ++i) {
        Wheel& wheel = wheels_[i];
        const OutputSettings& output = settings.outputs[i];

        // A 16-bit difference wraps correctly on any timer width.
        const auto delta = static_cast<std::int16_t>(static_cast<std::uint16_t>(counts[i] - wheel.last_count));
        const float counts_moved = wheel.primed ? static_cast<float>(output.encoder_reversed ? -delta : delta) : 0.0F;
        wheel.last_count = counts[i];
        wheel.primed = true;
        const float turned = counts_moved * 2.0F * std::numbers::pi_v<float> / output.counts_per_turn;
        wheel.out.angle += turned;
        wheel.out.speed = dt_s > 0.0F ? turned / dt_s : 0.0F;

        const float requested = i < targets.size() ? targets[i] : std::numeric_limits<float>::quiet_NaN();
        if (!enabled || std::isnan(requested)) {
            wheel.out.target = std::numeric_limits<float>::quiet_NaN();
            wheel.out.duty = 0.0F;
            wheel.integral = 0.0F;
            continue;
        }
        const float target = std::clamp(requested, -loop.max_speed, loop.max_speed);
        const float error = target - wheel.out.speed;
        // The window the duty may take: the output limit, narrowed by the slew
        // limit around last cycle's duty, in the wheel's own direction.
        float low = -loop.out_max;
        float high = loop.out_max;
        if (loop.slew_rate > 0.0F && i < previous.size()) {
            const float last = output.reversed ? -previous[i] : previous[i];
            const float step = loop.slew_rate * dt_s;
            low = std::max(low, std::min(last - step, high));
            high = std::min(high, std::max(last + step, low));
        }
        float integral = wheel.integral + loop.i * error * dt_s;
        float duty = loop.ff * target + loop.p * error + integral;
        // Clamping anti-windup: the integral stops growing towards a clamped side.
        if (duty > high) {
            duty = high;
            integral = error > 0.0F ? wheel.integral : integral;
        } else if (duty < low) {
            duty = low;
            integral = error < 0.0F ? wheel.integral : integral;
        }
        wheel.integral = integral;
        wheel.out.target = target;
        wheel.out.duty = output.reversed ? -duty : duty;
    }
}

}  // namespace vk::actuators
