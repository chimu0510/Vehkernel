// SPDX-License-Identifier: GPL-3.0-or-later
// Three-component vectors for estimation (ADR-0014): plain values in single
// precision, which the Cortex-M4 FPU computes in hardware.
#pragma once

#include <array>
#include <cmath>

namespace vk::estimation {

struct Vector3 {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;

    constexpr Vector3() = default;
    constexpr Vector3(float x0, float y0, float z0) : x(x0), y(y0), z(z0) {}

    static constexpr Vector3 from(const std::array<float, 3>& v) { return {v[0], v[1], v[2]}; }
    [[nodiscard]] constexpr std::array<float, 3> array() const { return {x, y, z}; }

    constexpr Vector3& operator+=(const Vector3& o)
    {
        x += o.x;
        y += o.y;
        z += o.z;
        return *this;
    }

    constexpr Vector3& operator-=(const Vector3& o)
    {
        x -= o.x;
        y -= o.y;
        z -= o.z;
        return *this;
    }

    constexpr Vector3& operator*=(float s)
    {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }

    [[nodiscard]] constexpr float dot(const Vector3& o) const { return x * o.x + y * o.y + z * o.z; }

    [[nodiscard]] constexpr Vector3 cross(const Vector3& o) const
    {
        return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
    }

    [[nodiscard]] float norm() const { return std::sqrt(dot(*this)); }

    // The unit vector in the same direction; the zero vector stays zero.
    [[nodiscard]] Vector3 normalized() const
    {
        const float n = norm();
        return n > 0.0F ? Vector3{x / n, y / n, z / n} : Vector3{};
    }
};

constexpr Vector3 operator+(Vector3 a, const Vector3& b)
{
    return a += b;
}

constexpr Vector3 operator-(Vector3 a, const Vector3& b)
{
    return a -= b;
}

constexpr Vector3 operator-(const Vector3& a)
{
    return {-a.x, -a.y, -a.z};
}

constexpr Vector3 operator*(Vector3 a, float s)
{
    return a *= s;
}

constexpr Vector3 operator*(float s, Vector3 a)
{
    return a *= s;
}

}  // namespace vk::estimation
