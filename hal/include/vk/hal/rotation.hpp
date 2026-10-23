// SPDX-License-Identifier: GPL-3.0-or-later
// Rotations of a board device (ADR-0014): the mounting rotation of a device
// on the board, a hardware fact the board generator emits, and the board
// orientation in the vehicle, the SENS_BOARD_ROT parameter. Names and numbers
// follow ArduPilot's ROTATION_* table, which PX4's SENS_BOARD_ROT shares, so
// parameter values mean the same there.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace vk::hal {

enum class Rotation : std::uint8_t {
    None = 0,
    Yaw45 = 1,
    Yaw90 = 2,
    Yaw135 = 3,
    Yaw180 = 4,
    Yaw225 = 5,
    Yaw270 = 6,
    Yaw315 = 7,
    Roll180 = 8,
    Roll180Yaw45 = 9,
    Roll180Yaw90 = 10,
    Roll180Yaw135 = 11,
    Pitch180 = 12,
    Roll180Yaw225 = 13,
    Roll180Yaw270 = 14,
    Roll180Yaw315 = 15,
    Roll90 = 16,
    Roll270 = 20,
    Pitch90 = 24,
    Pitch270 = 25,
};

// Every rotation there is, in numeric order.
inline constexpr std::array<Rotation, 20> all_rotations{
    Rotation::None,          Rotation::Yaw45,         Rotation::Yaw90,         Rotation::Yaw135,
    Rotation::Yaw180,        Rotation::Yaw225,        Rotation::Yaw270,        Rotation::Yaw315,
    Rotation::Roll180,       Rotation::Roll180Yaw45,  Rotation::Roll180Yaw90,  Rotation::Roll180Yaw135,
    Rotation::Pitch180,      Rotation::Roll180Yaw225, Rotation::Roll180Yaw270, Rotation::Roll180Yaw315,
    Rotation::Roll90,        Rotation::Roll270,       Rotation::Pitch90,       Rotation::Pitch270,
};

// Rotates a sensor-frame vector into the body frame for a device mounted
// with `rotation`. Roll180YawN means roll 180 degrees, then yaw N degrees.
constexpr std::array<float, 3> rotate(Rotation rotation, const std::array<float, 3>& v)
{
    constexpr float h = 0.70710678F;  // sin(45 deg) = cos(45 deg)
    const float x = v[0];
    const float y = v[1];
    const float z = v[2];
    switch (rotation) {
    case Rotation::None: return {x, y, z};
    case Rotation::Yaw45: return {h * (x - y), h * (x + y), z};
    case Rotation::Yaw90: return {-y, x, z};
    case Rotation::Yaw135: return {-h * (x + y), h * (x - y), z};
    case Rotation::Yaw180: return {-x, -y, z};
    case Rotation::Yaw225: return {h * (y - x), -h * (x + y), z};
    case Rotation::Yaw270: return {y, -x, z};
    case Rotation::Yaw315: return {h * (x + y), h * (y - x), z};
    case Rotation::Roll180: return {x, -y, -z};
    case Rotation::Roll180Yaw45: return {h * (x + y), h * (x - y), -z};
    case Rotation::Roll180Yaw90: return {y, x, -z};
    case Rotation::Roll180Yaw135: return {h * (y - x), h * (y + x), -z};
    case Rotation::Pitch180: return {-x, y, -z};
    case Rotation::Roll180Yaw225: return {-h * (x + y), h * (y - x), -z};
    case Rotation::Roll180Yaw270: return {-y, -x, -z};
    case Rotation::Roll180Yaw315: return {h * (x - y), -h * (x + y), -z};
    case Rotation::Roll90: return {x, -z, y};
    case Rotation::Roll270: return {x, z, -y};
    case Rotation::Pitch90: return {z, y, -x};
    case Rotation::Pitch270: return {-z, y, x};
    }
    return {x, y, z};
}

// A rotation as a matrix: column i is where axis i goes.
struct RotationMatrix {
    std::array<std::array<float, 3>, 3> m;  // m[row][column]

    [[nodiscard]] constexpr std::array<float, 3> operator*(const std::array<float, 3>& v) const
    {
        return {m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
                m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
                m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2]};
    }
};

// `first`, then `second`, as one matrix: a device mounted with `first` on a
// board that sits in the vehicle with `second` reads in the body frame as
// compose(first, second) * v.
constexpr RotationMatrix compose(Rotation first, Rotation second)
{
    RotationMatrix result{};
    for (std::size_t axis = 0; axis < 3; ++axis) {
        std::array<float, 3> unit{};
        unit[axis] = 1.0F;
        const std::array<float, 3> column = rotate(second, rotate(first, unit));
        for (std::size_t row = 0; row < 3; ++row) {
            result.m[row][axis] = column[row];
        }
    }
    return result;
}

}  // namespace vk::hal
