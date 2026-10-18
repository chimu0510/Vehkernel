// Mounting rotation of a board device relative to the body frame (ADR-0014).
// The names follow ArduPilot's ROTATION_* values; the board generator emits them.
#pragma once

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

}  // namespace vk::hal
