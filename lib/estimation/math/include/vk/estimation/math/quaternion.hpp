// Unit quaternions for attitude (ADR-0014).
//
// A quaternion q maps vectors from the body frame (x forward, y right,
// z down) to the local level frame (north-east-down): v_level = q v_body q*.
// Euler angles follow the aerospace ZYX convention: yaw, then pitch, then
// roll.
#pragma once

#include <vk/estimation/math/vector3.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

namespace vk::estimation {

struct Euler {
    float roll = 0.0F;   // [rad] about x
    float pitch = 0.0F;  // [rad] about y
    float yaw = 0.0F;    // [rad] about z
};

struct Quaternion {
    float w = 1.0F;
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;

    constexpr Quaternion() = default;
    constexpr Quaternion(float w0, float x0, float y0, float z0) : w(w0), x(x0), y(y0), z(z0) {}

    static constexpr Quaternion identity() { return {}; }

    static constexpr Quaternion from(const std::array<float, 4>& q) { return {q[0], q[1], q[2], q[3]}; }
    [[nodiscard]] constexpr std::array<float, 4> array() const { return {w, x, y, z}; }

    static Quaternion from_euler(const Euler& e)
    {
        const float cr = std::cos(e.roll / 2.0F);
        const float sr = std::sin(e.roll / 2.0F);
        const float cp = std::cos(e.pitch / 2.0F);
        const float sp = std::sin(e.pitch / 2.0F);
        const float cy = std::cos(e.yaw / 2.0F);
        const float sy = std::sin(e.yaw / 2.0F);
        return {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy};
    }

    // The rotation by |v| radians about v: exp(v / 2).
    static Quaternion from_rotation_vector(const Vector3& v)
    {
        const float angle = v.norm();
        if (angle < 1e-6F) {
            // First order, exact to the precision of a float at this size.
            return Quaternion{1.0F, v.x / 2.0F, v.y / 2.0F, v.z / 2.0F}.normalized();
        }
        const float s = std::sin(angle / 2.0F) / angle;
        return {std::cos(angle / 2.0F), v.x * s, v.y * s, v.z * s};
    }

    // Roll, pitch and yaw. At pitch ±90° roll and yaw turn about the same
    // axis; roll is then 0 and yaw carries the whole turn.
    [[nodiscard]] Euler euler() const
    {
        const float sin_pitch = std::clamp(2.0F * (w * y - z * x), -1.0F, 1.0F);
        if (std::abs(sin_pitch) > 0.99999F) {
            return {.roll = 0.0F,
                    .pitch = std::copysign(std::numbers::pi_v<float> / 2.0F, sin_pitch),
                    .yaw = 2.0F * std::atan2(z, w)};
        }
        return {.roll = std::atan2(2.0F * (w * x + y * z), 1.0F - 2.0F * (x * x + y * y)),
                .pitch = std::asin(sin_pitch),
                .yaw = std::atan2(2.0F * (w * z + x * y), 1.0F - 2.0F * (y * y + z * z))};
    }

    [[nodiscard]] constexpr Quaternion conjugate() const { return {w, -x, -y, -z}; }

    [[nodiscard]] float norm() const { return std::sqrt(w * w + x * x + y * y + z * z); }

    [[nodiscard]] Quaternion normalized() const
    {
        const float n = norm();
        return n > 0.0F ? Quaternion{w / n, x / n, y / n, z / n} : identity();
    }

    // Hamilton product: the rotation `o`, then this one.
    [[nodiscard]] constexpr Quaternion operator*(const Quaternion& o) const
    {
        return {w * o.w - x * o.x - y * o.y - z * o.z, w * o.x + x * o.w + y * o.z - z * o.y,
                w * o.y - x * o.z + y * o.w + z * o.x, w * o.z + x * o.y - y * o.x + z * o.w};
    }

    // A body-frame vector in the level frame.
    [[nodiscard]] constexpr Vector3 rotate(const Vector3& v) const
    {
        const Vector3 u{x, y, z};
        const Vector3 t = 2.0F * u.cross(v);
        return v + w * t + u.cross(t);
    }

    // A level-frame vector in the body frame.
    [[nodiscard]] constexpr Vector3 inverse_rotate(const Vector3& v) const { return conjugate().rotate(v); }

    // The attitude after turning at the body rate `omega` [rad/s] for `dt` seconds.
    [[nodiscard]] Quaternion integrated(const Vector3& omega, float dt) const
    {
        return (*this * from_rotation_vector(omega * dt)).normalized();
    }

    // The angle of the rotation from this attitude to `o` [rad], in [0, pi].
    [[nodiscard]] float angle_to(const Quaternion& o) const
    {
        const Quaternion d = conjugate() * o;
        return 2.0F * std::atan2(Vector3{d.x, d.y, d.z}.norm(), std::abs(d.w));
    }
};

}  // namespace vk::estimation
