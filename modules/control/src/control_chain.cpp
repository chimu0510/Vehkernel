// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/control/control_chain.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace vk::control {

void ControlChain::reset()
{
    yaw_integral_ = 0.0F;
    limited_ = 0;
    scale_ = 1.0F;
}

ChainResult ControlChain::step(float dt_s, const Twist& setpoint, std::optional<float> measured_yaw_rate,
                               bool closed, const RateSettings& settings, float max_speed, OutputTargets targets)
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
    // Velocity control: vx and vy as feedforward.
    result.demand = {.vx = setpoint.vx, .vy = setpoint.vy, .yaw_rate = settings.ff * setpoint.yaw_rate};
    // Angular rate control on the gyro.
    if (closed && measured_yaw_rate && std::isfinite(*measured_yaw_rate)) {
        const float error = setpoint.yaw_rate * scale_ - *measured_yaw_rate;
        if ((limited_ & static_cast<std::uint8_t>(TwistAxis::YawRate)) == 0) {
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
