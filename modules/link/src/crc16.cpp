#include <vk/link/crc16.hpp>

namespace vk::link {

std::uint16_t crc16(std::span<const std::uint8_t> data)
{
    std::uint16_t crc = 0xFFFF;
    for (const std::uint8_t byte : data) {
        crc = static_cast<std::uint16_t>(crc ^ (byte << 8U));
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000U) != 0 ? static_cast<std::uint16_t>((crc << 1U) ^ 0x1021U)
                                       : static_cast<std::uint16_t>(crc << 1U);
        }
    }
    return crc;
}

}  // namespace vk::link
