// CRC-16/CCITT-FALSE (polynomial 0x1021, initial value 0xFFFF, no reflection,
// no final XOR), which protects every Link frame (ADR-0002).
#pragma once

#include <cstdint>
#include <span>

namespace vk::link {

std::uint16_t crc16(std::span<const std::uint8_t> data);

}  // namespace vk::link
