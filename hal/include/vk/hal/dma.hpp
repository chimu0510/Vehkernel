// SPDX-License-Identifier: GPL-3.0-or-later
// DMA assignments emitted by the board generator (ADR-0011).
#pragma once

#include <cstdint>

namespace vk::hal {

enum class DmaController : std::uint8_t { Dma1 = 1, Dma2 = 2 };

// A DMA stream and channel solved for one request. Shared streams are only
// produced when the board definition allowed sharing for every user.
template <DmaController C, std::uint8_t Stream, std::uint8_t Channel, bool Shared>
struct DmaStream {
    static constexpr DmaController controller = C;
    static constexpr std::uint8_t stream = Stream;
    static constexpr std::uint8_t channel = Channel;
    static constexpr bool shared = Shared;
};

}  // namespace vk::hal
