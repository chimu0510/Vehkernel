// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/link/crc16.hpp>

#include <array>

namespace vk::link {
namespace {

// The CRC of each byte value, one table lookup per byte instead of eight
// shifts: every frame the Link sends and receives goes through it (#178).
constexpr std::array<std::uint16_t, 256> crc16_table = [] {
    std::array<std::uint16_t, 256> table{};
    for (std::size_t value = 0; value < table.size(); ++value) {
        auto crc = static_cast<std::uint16_t>(value << 8U);
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000U) != 0 ? static_cast<std::uint16_t>((crc << 1U) ^ 0x1021U)
                                       : static_cast<std::uint16_t>(crc << 1U);
        }
        table[value] = crc;
    }
    return table;
}();

}  // namespace

std::uint16_t crc16(std::span<const std::uint8_t> data)
{
    std::uint16_t crc = 0xFFFF;
    for (const std::uint8_t byte : data) {
        crc = static_cast<std::uint16_t>((crc << 8U) ^ crc16_table[((crc >> 8U) ^ byte) & 0xFFU]);
    }
    return crc;
}

}  // namespace vk::link
