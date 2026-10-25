// SPDX-License-Identifier: GPL-3.0-or-later
// The WHEELED vehicle model (ADR-0013): differential and skid-steer frames
// on the generic wheel matrix of lib/estimation.
//
// The frame type places the wheels from the radius and the track: DIFF_2WD
// has a left and a right wheel on the body origin's axle; SKID_4WD a front
// and a rear pair, which plain wheels drive the same way, so the wheelbase
// does not enter the math. Each wheel drives the actuator output its
// WHL_*_OUT parameter names. The model is invalid when the geometry is not
// (a radius or track that is not positive) or two wheels share an output.
#pragma once

#include <vk/estimation/models/wheel_kinematics.hpp>
#include <vk/msg/enums.hpp>
#include <vk/vehicle_model/vehicle_model.hpp>

#include <array>
#include <cstddef>
#include <cstdint>

namespace vk::vehicle_model {

struct WheeledFrame {
    msg::FrameType type = msg::FrameType::Diff2wd;
    float radius = 0.05F;  // [m] WHL_RADIUS
    float track = 0.2F;    // [m] WHL_TRACK
    // The output of each wheel, 1 for output 1: left, right (the front pair
    // for SKID_4WD), then rear left and rear right.
    std::array<std::uint8_t, 4> outputs{1, 2, 3, 4};
};

class WheeledModel final : public VehicleModel {
public:
    WheeledModel() = default;
    explicit WheeledModel(const WheeledFrame& frame) { set(frame); }

    void set(const WheeledFrame& frame);

    [[nodiscard]] bool valid() const { return valid_; }
    [[nodiscard]] const estimation::WheelKinematics& kinematics() const { return kinematics_; }
    // The output index (0 for output 1) of wheel `wheel`.
    [[nodiscard]] std::size_t output(std::size_t wheel) const { return outputs_.at(wheel); }

    [[nodiscard]] WheelAllocation allocate(const Twist& demand, float max_speed, OutputTargets targets) const override;
    [[nodiscard]] std::optional<WheelMeasurement> measure(OutputSpeeds speeds) const override;

private:
    estimation::WheelKinematics kinematics_;
    std::array<std::size_t, estimation::max_wheels> outputs_{};
    bool valid_ = false;
};

}  // namespace vk::vehicle_model
