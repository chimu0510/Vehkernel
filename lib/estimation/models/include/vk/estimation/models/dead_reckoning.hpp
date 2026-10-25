// SPDX-License-Identifier: GPL-3.0-or-later
// Process model: dead reckoning in the local level plane (ADR-0014), the
// local odometry's position from body velocity measurements and a heading.
//
// The heading comes from the attitude estimator, never from the wheels'
// speed difference, so a skid does not turn the track (ADR-0014). Each step
// moves the position by the body velocity rotated by the step's mid heading
// (north-east, heading from north, positive turning right):
//
//   north += (vx·cos ψ - vy·sin ψ)·dt     east += (vx·sin ψ + vy·cos ψ)·dt
//
// The position's covariance grows by the velocity's variance rotated into
// the level plane, plus the heading's variance carried by the speed, both
// times dt²; the velocity's variance includes a share of the speed itself
// (the scale error of wheel radii and encoders), so the uncertainty grows
// with the distance driven and with the measurement's residual; at rest only
// by the measurement's own small noise.
#pragma once

#include <array>
#include <cmath>
#include <numbers>

namespace vk::estimation {

class DeadReckoning {
public:
    // The relative speed error assumed of every body velocity measurement,
    // as a standard deviation: 2 % of the speed.
    static constexpr float scale_error = 0.02F;

    // One body velocity measurement `dt_s` after the last: vx and vy [m/s] in
    // the body frame with their variances [m²/s²], and the heading [rad] at
    // its time with its variance [rad²].
    void step(float dt_s, float vx, float vy, float var_vx, float var_vy, float heading, float var_heading)
    {
        const float mid = has_heading_ ? heading_ + std::remainder(heading - heading_, 2.0F * pi) / 2.0F : heading;
        heading_ = heading;
        has_heading_ = true;
        if (!(dt_s > 0.0F)) {
            return;
        }
        const float c = std::cos(mid);
        const float s = std::sin(mid);
        north_ += static_cast<double>((vx * c - vy * s) * dt_s);
        east_ += static_cast<double>((vx * s + vy * c) * dt_s);
        distance_ += std::hypot(vx, vy) * dt_s;

        // Velocity variance with the scale error, rotated into the level plane.
        const float sx = var_vx + scale_error * scale_error * vx * vx;
        const float sy = var_vy + scale_error * scale_error * vy * vy;
        const float dt2 = dt_s * dt_s;
        covariance_[0] += (c * c * sx + s * s * sy) * dt2;  // north-north
        covariance_[1] += (c * s * (sx - sy)) * dt2;        // north-east
        covariance_[2] += (s * s * sx + c * c * sy) * dt2;  // east-east
        // The heading's variance, along the direction the heading turns the motion.
        const float gn = -vx * s - vy * c;
        const float ge = vx * c - vy * s;
        covariance_[0] += gn * gn * var_heading * dt2;
        covariance_[1] += gn * ge * var_heading * dt2;
        covariance_[2] += ge * ge * var_heading * dt2;
    }

    [[nodiscard]] double north() const { return north_; }
    [[nodiscard]] double east() const { return east_; }
    [[nodiscard]] float distance() const { return distance_; }  // [m] driven since the origin
    // The position's covariance [m²]: north-north, north-east, east-east.
    [[nodiscard]] const std::array<float, 3>& covariance() const { return covariance_; }

private:
    static constexpr float pi = std::numbers::pi_v<float>;

    double north_ = 0.0;
    double east_ = 0.0;
    float distance_ = 0.0F;
    float heading_ = 0.0F;
    bool has_heading_ = false;
    std::array<float, 3> covariance_{};
};

}  // namespace vk::estimation
