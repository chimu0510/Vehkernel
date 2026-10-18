// Measurement model: the accelerometer at rest (ADR-0014).
//
// An accelerometer measures specific force: acceleration minus gravity. At
// rest in the north-east-down level frame that is (0, 0, -g), pointing up;
// in the body frame it depends only on roll and pitch, which is what an
// attitude filter corrects from it.
#pragma once

#include <vk/estimation/math/quaternion.hpp>
#include <vk/estimation/math/vector3.hpp>

namespace vk::estimation {

inline constexpr float standard_gravity = 9.80665F;  // [m/s^2]

struct AccelerometerModel {
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
};

}  // namespace vk::estimation
