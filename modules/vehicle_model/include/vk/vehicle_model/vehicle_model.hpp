// SPDX-License-Identifier: GPL-3.0-or-later
// The vehicle model (ADR-0013): the last step of the control chain before
// actuator control, and the only place frames differ. Every model
// implements this interface; one is selected at boot by FRAME_CLASS
// (ADR-0009: a runtime choice, so one firmware image drives any supported
// frame on the board).
//
// Allocation turns a body motion demand into a wheel speed target per
// actuator output, scaled down uniformly past the largest wheel speed, and
// reports the axes the scaling limited so the controllers stop integrating
// on them. A model with wheel speed feedback also measures the body velocity
// from the outputs' wheel speeds (ADR-0014). Outputs the model does not
// drive get NaN: uncontrolled, as actuator control takes it.
#pragma once

#include <vk/estimation/models/wheel_kinematics.hpp>
#include <vk/msg/actuator_setpoint.hpp>

#include <cstddef>
#include <optional>
#include <span>
#include <tuple>

namespace vk::vehicle_model {

using estimation::Twist;
using estimation::TwistAxis;
using estimation::WheelAllocation;
using estimation::WheelMeasurement;

// One per actuator output: the length of the actuator topics' arrays.
inline constexpr std::size_t max_outputs = std::tuple_size_v<decltype(msg::ActuatorSetpoint::wheel_speed)>;

using OutputTargets = std::span<float, max_outputs>;
using OutputSpeeds = std::span<const float, max_outputs>;

class VehicleModel {
public:
    VehicleModel() = default;
    VehicleModel(const VehicleModel&) = delete;
    VehicleModel& operator=(const VehicleModel&) = delete;
    VehicleModel(VehicleModel&&) = delete;
    VehicleModel& operator=(VehicleModel&&) = delete;

    // The wheel speed target of each output [rad/s] for `demand`, none above
    // `max_speed`; NaN for outputs the model does not drive, and for every
    // output when the demand is not finite.
    [[nodiscard]] virtual WheelAllocation allocate(const Twist& demand, float max_speed,
                                                   OutputTargets targets) const = 0;

    // The body velocity the outputs' wheel speeds [rad/s] show, when the
    // model has wheel speed feedback.
    [[nodiscard]] virtual std::optional<WheelMeasurement> measure(OutputSpeeds speeds) const = 0;

protected:
    ~VehicleModel() = default;
};

}  // namespace vk::vehicle_model
