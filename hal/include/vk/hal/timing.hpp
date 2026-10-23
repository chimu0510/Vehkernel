// SPDX-License-Identifier: GPL-3.0-or-later
// Timing preset dimensions (ADR-0015); the board generator emits them.
#pragma once

#include <cstdint>

namespace vk::hal {

// What starts the control path: an IMU interrupt or a fixed-rate timer.
enum class TimingTrigger : std::uint8_t {
    ImuEvent,
    Timer,
};

// How the IMU driver reads samples: one at a time or in FIFO batches.
enum class ImuSampling : std::uint8_t {
    PerSample,
    Fifo,
};

}  // namespace vk::hal
