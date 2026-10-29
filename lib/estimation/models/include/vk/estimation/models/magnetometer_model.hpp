// SPDX-License-Identifier: GPL-3.0-or-later
// Measurement model: the magnetometer as a heading measurement (ADR-0014).
//
// The body-frame magnetic field is levelled with the estimate's roll and
// pitch, its magnetic heading is taken from north and east and compared with
// the estimate's yaw minus the magnetic declination, wrapped to [-pi, pi].
// Its Jacobian touches the yaw error only, so a disturbed field can never
// tilt roll and pitch.
#pragma once

#include <vk/estimation/math/matrix.hpp>
#include <vk/estimation/math/quaternion.hpp>
#include <vk/estimation/math/vector3.hpp>

#include <cmath>
#include <numbers>
#include <optional>

namespace vk::estimation {

struct HeadingMeasurement {
    float innovation = 0.0F;
    Matrix<1, 6> jacobian{};
    float variance = 0.0F;
};

struct MagnetometerModel {
    using Measurement = HeadingMeasurement;

    // Wraps an angle [rad] to [-pi, pi].
    [[nodiscard]] static float wrap_pi(float angle)
    {
        return std::remainder(angle, 2.0F * std::numbers::pi_v<float>);
    }

    // The Earth magnetic field (in the north-east-down level frame) seen in the body frame.
    [[nodiscard]] static constexpr Vector3 predict(const Quaternion& q, const Vector3& earth_field)
    {
        return q.inverse_rotate(earth_field);
    }

    // Yaw [rad] from a body-frame field levelled by roll and pitch, plus magnetic declination [rad].
    [[nodiscard]] static float heading_from(float roll, float pitch, const Vector3& field, float declination = 0.0F)
    {
        const Quaternion level = Quaternion::from_euler({.roll = roll, .pitch = pitch, .yaw = 0.0F});
        const Vector3 levelled = level.rotate(field);
        const float magnetic_heading = std::atan2(-levelled.y, levelled.x);
        return wrap_pi(magnetic_heading + declination);
    }

    [[nodiscard]] static float heading_from(const Quaternion& q, const Vector3& field, float declination = 0.0F)
    {
        const Euler e = q.euler();
        return heading_from(e.roll, e.pitch, field, declination);
    }

    // True when the field strength differs from reference_strength by at most mag_rejection * reference_strength.
    [[nodiscard]] static bool within_field_strength(const Vector3& field, float reference_strength, float mag_rejection)
    {
        const float strength = field.norm();
        return strength > 0.0F && reference_strength > 0.0F &&
               std::abs(strength - reference_strength) <= mag_rejection * reference_strength;
    }

    // The heading measurement: levelled heading compared with (yaw - declination), wrapped to [-pi, pi],
    // with a Jacobian on the yaw error only and variance mag_noise^2 [rad^2].
    [[nodiscard]] static HeadingMeasurement heading_measurement(const Quaternion& q, const Vector3& field,
                                                                float mag_noise, float declination = 0.0F)
    {
        const Euler e = q.euler();
        const Quaternion level = Quaternion::from_euler({.roll = e.roll, .pitch = e.pitch, .yaw = 0.0F});
        const Vector3 levelled = level.rotate(field);
        const float magnetic_heading = std::atan2(-levelled.y, levelled.x);
        const float innovation = wrap_pi(magnetic_heading - (e.yaw - declination));
        return {
            .innovation = innovation,
            .jacobian = Matrix<1, 6>::from_rows({{{{0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F}}}}),
            .variance = mag_noise * mag_noise,
        };
    }

    // Returns the heading measurement when the horizontal field is non-zero and (if reference_strength > 0)
    // within mag_rejection of reference_strength, or nullopt to skip it.
    [[nodiscard]] static std::optional<HeadingMeasurement> measure_heading(const Quaternion& q, const Vector3& field,
                                                                           float mag_noise, float declination = 0.0F,
                                                                           float reference_strength = 0.0F,
                                                                           float mag_rejection = 0.2F)
    {
        if (reference_strength > 0.0F && !within_field_strength(field, reference_strength, mag_rejection)) {
            return std::nullopt;
        }
        const Euler e = q.euler();
        const Quaternion level = Quaternion::from_euler({.roll = e.roll, .pitch = e.pitch, .yaw = 0.0F});
        const Vector3 levelled = level.rotate(field);
        if (!(levelled.x * levelled.x + levelled.y * levelled.y > 0.0F)) {
            return std::nullopt;
        }
        return heading_measurement(q, field, mag_noise, declination);
    }
};

}  // namespace vk::estimation
