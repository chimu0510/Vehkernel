// Mounting rotation of a board device relative to the body frame (ADR-0014).
// The names follow ArduPilot's ROTATION_* values; the board generator emits them.
#pragma once

#include <array>
#include <cstdint>

namespace vk::hal {

enum class Rotation : std::uint8_t {
    None,
    Yaw45,
    Yaw90,
    Yaw135,
    Yaw180,
    Yaw225,
    Yaw270,
    Yaw315,
    Roll180,
    Roll180Yaw45,
    Roll180Yaw90,
    Roll180Yaw135,
    Pitch180,
    Roll180Yaw225,
    Roll180Yaw270,
    Roll180Yaw315,
    Roll90,
    Roll270,
    Pitch90,
    Pitch270,
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

}  // namespace vk::hal
