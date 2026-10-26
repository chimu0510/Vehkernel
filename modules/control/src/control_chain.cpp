// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/control/control_chain.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace vk::control {
namespace {

// `current` moved towards `target` by at most `acc_max`·dt while its size
// grows and `dec_max`·dt while it shrinks; across zero it stops at zero
// first. A limit of 0 does not limit.
float limit_rate(float current, float target, float acc_max, float dec_max, float dt_s)
{
    if (acc_max <= 0.0F && dec_max <= 0.0F) {
        return target;
    }
    const bool slowing = current * target < 0.0F || std::fabs(target) < std::fabs(current);
    const float goal = current * target < 0.0F ? 0.0F : target;
    const float rate = slowing ? dec_max : acc_max;
    if (rate <= 0.0F) {
        return goal;
    }
    const float step = rate * dt_s;
    return current + std::clamp(goal - current, -step, step);
}

}  // namespace

void ControlChain::reset()
{
    yaw_integral_ = 0.0F;
    limited_ = 0;
    scale_ = 1.0F;
    vx_ = 0.0F;
    vy_ = 0.0F;
}

ChainResult ControlChain::step(float dt_s, const Twist& setpoint, std::optional<float> measured_yaw_rate,
                               bool closed, const RateSettings& settings, float max_speed, OutputTargets targets,
                               const VelocityLimits& limits)
{
    ChainResult result{};
    const bool finite = std::isfinite(setpoint.vx) && std::isfinite(setpoint.vy) && std::isfinite(setpoint.yaw_rate);
    if (model_ == nullptr || !finite) {
        reset();
        std::ranges::fill(targets, std::numeric_limits<float>::quiet_NaN());
        return result;
    }
    if (!closed) {
        reset();
    }
    // Velocity control: vx and vy as feedforward, their rate of change
    // limited while the loops close.
    Twist limited = setpoint;
    if (closed) {
        vx_ = limit_rate(vx_, setpoint.vx, limits.acc_max, limits.dec_max, dt_s);
        vy_ = limit_rate(vy_, setpoint.vy, limits.acc_max, limits.dec_max, dt_s);
        limited.vx = vx_;
        limited.vy = vy_;
    }
    const bool at_rest = limited.vx == 0.0F && limited.vy == 0.0F && setpoint.yaw_rate == 0.0F;
    if (at_rest) {
        yaw_integral_ = 0.0F;
    }
    result.demand = {.vx = limited.vx, .vy = limited.vy, .yaw_rate = settings.ff * setpoint.yaw_rate};
    // Angular rate control on the gyro.
    if (closed && measured_yaw_rate && std::isfinite(*measured_yaw_rate)) {
        const float error = setpoint.yaw_rate * scale_ - *measured_yaw_rate;
        if (!at_rest && (limited_ & static_cast<std::uint8_t>(TwistAxis::YawRate)) == 0) {
            yaw_integral_ += settings.i * error * dt_s;
        }
        result.demand.yaw_rate += settings.p * error + yaw_integral_;
    } else if (closed) {
        result.demand.yaw_rate += yaw_integral_;
    }
    result.allocation = model_->allocate(result.demand, max_speed, targets);
    limited_ = closed ? result.allocation.limited : 0;
    scale_ = closed ? result.allocation.scale : 1.0F;
    return result;
}

}  // namespace vk::control
