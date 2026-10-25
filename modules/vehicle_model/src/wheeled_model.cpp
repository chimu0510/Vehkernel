// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/vehicle_model/wheeled_model.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace vk::vehicle_model {

void WheeledModel::set(const WheeledFrame& frame)
{
    valid_ = false;
    const float half = frame.track / 2.0F;
    std::array<estimation::WheelGeometry, estimation::max_wheels> wheels{};
    std::size_t count = 0;
    switch (frame.type) {
    case msg::FrameType::Diff2wd:
        wheels[0] = {.y = -half, .radius = frame.radius};
        wheels[1] = {.y = half, .radius = frame.radius};
        count = 2;
        break;
    case msg::FrameType::Skid4wd:
        // Plain wheels: the wheelbase does not enter J, so the pairs sit on
        // the origin's axle.
        wheels[0] = {.y = -half, .radius = frame.radius};
        wheels[1] = {.y = half, .radius = frame.radius};
        wheels[2] = {.y = -half, .radius = frame.radius};
        wheels[3] = {.y = half, .radius = frame.radius};
        count = 4;
        break;
    }
    if (count == 0 || !(frame.track > 0.0F)) {
        return;
    }
    std::uint8_t used = 0;
    for (std::size_t i = 0; i < count; ++i) {
        const std::uint8_t output = frame.outputs.at(i);
        if (output < 1 || output > max_outputs || (used & (1U << (output - 1U))) != 0) {
            return;
        }
        used = static_cast<std::uint8_t>(used | (1U << (output - 1U)));
        outputs_.at(i) = output - 1U;
    }
    kinematics_.set(std::span(wheels).first(count));
    valid_ = kinematics_.valid();
}

WheelAllocation WheeledModel::allocate(const Twist& demand, float max_speed, OutputTargets targets) const
{
    std::ranges::fill(targets, std::numeric_limits<float>::quiet_NaN());
    if (!valid_ || !std::isfinite(demand.vx) || !std::isfinite(demand.vy) || !std::isfinite(demand.yaw_rate)) {
        return {};
    }
    std::array<float, estimation::max_wheels> speeds{};
    const WheelAllocation result = kinematics_.allocate(demand, max_speed, speeds);
    for (std::size_t i = 0; i < kinematics_.size(); ++i) {
        targets[outputs_[i]] = speeds[i];
    }
    return result;
}

std::optional<WheelMeasurement> WheeledModel::measure(OutputSpeeds speeds, OutputSpeeds noise) const
{
    if (!valid_) {
        return std::nullopt;
    }
    std::array<float, estimation::max_wheels> wheels{};
    std::array<float, estimation::max_wheels> wheel_noise{};
    for (std::size_t i = 0; i < kinematics_.size(); ++i) {
        wheels[i] = speeds[outputs_[i]];
        wheel_noise[i] = noise[outputs_[i]];
        if (!std::isfinite(wheels[i])) {
            return std::nullopt;
        }
    }
    const std::size_t n = kinematics_.size();
    return kinematics_.measure(std::span(wheels).first(n), std::span(wheel_noise).first(n));
}

}  // namespace vk::vehicle_model
