// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/control/control_chain.hpp>

#include <algorithm>
#include <limits>

namespace vk::control {

WheelAllocation ControlChain::step(const Twist& setpoint, float max_speed, OutputTargets targets)
{
    if (model_ == nullptr) {
        std::ranges::fill(targets, std::numeric_limits<float>::quiet_NaN());
        return {};
    }
    // Velocity control: vx and vy as feedforward; the yaw rate as asked.
    return model_->allocate(setpoint, max_speed, targets);
}

}  // namespace vk::control
