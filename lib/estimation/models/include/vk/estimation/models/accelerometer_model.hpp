// SPDX-License-Identifier: GPL-3.0-or-later
// Measurement model: the accelerometer at rest (ADR-0014).
//
// An accelerometer measures specific force: acceleration minus gravity. At
// rest in the north-east-down level frame that is (0, 0, -g), pointing up;
// in the body frame it depends only on roll and pitch, which is what an
// attitude filter corrects from it.
#pragma once

#include <vk/estimation/math/matrix.hpp>
#include <vk/estimation/math/quaternion.hpp>
#include <vk/estimation/math/vector3.hpp>

#include <cmath>
#include <optional>

namespace vk::estimation {

inline constexpr float standard_gravity = 9.80665F;  // [m/s^2]

struct GravityMeasurement {
    Vector3 innovation{};
    Matrix<3, 6> jacobian{};
    float variance = 0.0F;
};

struct AccelerometerModel {
    using Measurement = GravityMeasurement;

    // The specific force at rest in the body frame [m/s^2].
    [[nodiscard]] static constexpr Vector3 predict(const Quaternion& q, float g = standard_gravity)
    {
        return q.inverse_rotate(Vector3{0.0F, 0.0F, -g});
    }

    // Its direction: the unit vector pointing up, in the body frame.
    [[nodiscard]] static constexpr Vector3 predict_direction(const Quaternion& q)
    {
        return q.inverse_rotate(Vector3{0.0F, 0.0F, -1.0F});
    }

    // Roll and pitch from a specific force measured at rest; yaw is not observable.
    [[nodiscard]] static Euler level_from(const Vector3& specific_force)
    {
        const Vector3 up = specific_force.normalized();
        // At rest the body measures (sin(pitch), -sin(roll)cos(pitch), -cos(roll)cos(pitch)).
        return {.roll = std::atan2(-up.y, -up.z), .pitch = std::atan2(up.x, std::sqrt(up.y * up.y + up.z * up.z))};
    }

    // True when the specific force differs from gravity by at most accel_rejection * g.
    [[nodiscard]] static bool within_gravity(const Vector3& specific_force, float accel_rejection,
                                             float g = standard_gravity)
    {
        const float magnitude = specific_force.norm();
        return magnitude > 0.0F && std::abs(magnitude - g) <= accel_rejection * g;
    }

    // The gravity measurement: the normalized specific force against the estimate's
    // vertical axis, with its Jacobian on the 6-state error (NED attitude error,
    // body gyro bias error) and its variance from accel_noise [m/s^2].
    [[nodiscard]] static GravityMeasurement gravity_measurement(const Quaternion& q, const Vector3& specific_force,
                                                                float accel_noise, float g = standard_gravity)
    {
        const Vector3 predicted = predict_direction(q);
        const Vector3 measured = specific_force.normalized();
        Matrix<3, 6> jacobian{};
        jacobian.set_block(0, 0, Matrix<3, 3>::skew(predicted) * Matrix<3, 3>::rotation(q).transposed());
        const float sigma = accel_noise / g;
        return {
            .innovation = measured - predicted,
            .jacobian = jacobian,
            .variance = sigma * sigma,
        };
    }

    // Returns the gravity measurement when within accel_rejection of g, or nullopt to skip it.
    [[nodiscard]] static std::optional<GravityMeasurement> measure_gravity(const Quaternion& q,
                                                                           const Vector3& specific_force,
                                                                           float accel_noise, float accel_rejection,
                                                                           float g = standard_gravity)
    {
        if (!within_gravity(specific_force, accel_rejection, g)) {
            return std::nullopt;
        }
        return gravity_measurement(q, specific_force, accel_noise, g);
    }
};

}  // namespace vk::estimation
