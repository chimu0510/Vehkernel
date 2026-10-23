// The check that a firmware image may start (ADR-0023), after ArduPilot's
// check_good_firmware_unsigned (libraries/AP_CheckFirmware): its app
// descriptor is present, its size fits, its board ID is the board's and its
// two CRCs match (core/include/vk/core/app_descriptor.hpp).
#pragma once

#include <vk/bootloader/platform.hpp>
#include <vk/core/app_descriptor.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>

namespace vk::bootloader {

enum class FirmwareCheck : std::uint8_t {
    Ok,
    NoDescriptor,  // FAIL_REASON_NO_APP_SIG
    BadLength,     // FAIL_REASON_BAD_LENGTH_APP
    BadBoardId,    // FAIL_REASON_BAD_BOARD_ID
    BadCrc,        // FAIL_REASON_BAD_CRC
};

constexpr std::string_view to_string(FirmwareCheck check)
{
    switch (check) {
    case FirmwareCheck::Ok:
        return "ok";
    case FirmwareCheck::NoDescriptor:
        return "no_descriptor";
    case FirmwareCheck::BadLength:
        return "bad_length";
    case FirmwareCheck::BadBoardId:
        return "bad_board_id";
    case FirmwareCheck::BadCrc:
        return "bad_crc";
    }
    return "?";
}

// ArduPilot's crc32_small: polynomial 0xEDB88320, no final inversion.
constexpr std::uint32_t crc32(std::uint32_t crc, std::uint8_t byte)
{
    crc ^= byte;
    for (int bit = 0; bit < 8; ++bit) {
        crc = (crc >> 1U) ^ ((crc & 1U) != 0U ? 0xEDB88320U : 0U);
    }
    return crc;
}

constexpr std::uint32_t crc32_word(std::uint32_t crc, std::uint32_t word)
{
    for (unsigned shift = 0; shift < 32; shift += 8) {
        crc = crc32(crc, static_cast<std::uint8_t>(word >> shift));
    }
    return crc;
}

namespace detail {

constexpr std::uint32_t le_word(const std::array<std::uint8_t, 8>& bytes, std::size_t at)
{
    return static_cast<std::uint32_t>(bytes[at]) | (static_cast<std::uint32_t>(bytes[at + 1]) << 8U) |
           (static_cast<std::uint32_t>(bytes[at + 2]) << 16U) | (static_cast<std::uint32_t>(bytes[at + 3]) << 24U);
}

// The descriptor's offset in the image: its signature, word-aligned as the linker places it.
template <Platform P>
std::optional<std::uint32_t> find_descriptor(const P& p)
{
    constexpr std::uint32_t first = le_word(core::app_descriptor_signature, 0);
    constexpr std::uint32_t second = le_word(core::app_descriptor_signature, 4);
    const std::uint32_t end = p.firmware_size() - sizeof(core::AppDescriptor);
    for (std::uint32_t offset = 0; offset <= end; offset += 4) {
        if (p.read_word(offset) == first && p.read_word(offset + 4) == second) {
            return offset;
        }
    }
    return std::nullopt;
}

// CRC of the image's bytes from `begin` to `end`, `begin` a multiple of 4.
template <Platform P>
std::uint32_t crc_range(const P& p, std::uint32_t begin, std::uint32_t end)
{
    std::uint32_t crc = 0;
    std::uint32_t offset = begin;
    for (; offset + 4 <= end; offset += 4) {
        crc = crc32_word(crc, p.read_word(offset));
    }
    if (offset < end) {
        const std::uint32_t word = p.read_word(offset);
        for (std::uint32_t i = 0; offset + i < end; ++i) {
            crc = crc32(crc, static_cast<std::uint8_t>(word >> (8U * i)));
        }
    }
    return crc;
}

}  // namespace detail

template <Platform P>
FirmwareCheck check_firmware(const P& p, std::uint16_t board_id)
{
    const std::optional<std::uint32_t> found = detail::find_descriptor(p);
    if (!found) {
        return FirmwareCheck::NoDescriptor;
    }
    const std::uint32_t at = *found;
    const std::uint32_t crc1 = p.read_word(at + offsetof(core::AppDescriptor, image_crc1));
    const std::uint32_t crc2 = p.read_word(at + offsetof(core::AppDescriptor, image_crc2));
    const std::uint32_t size = p.read_word(at + offsetof(core::AppDescriptor, image_size));
    const std::uint32_t version_word = p.read_word(at + offsetof(core::AppDescriptor, version_major));
    if (size > p.firmware_size() || size < at + sizeof(core::AppDescriptor)) {
        return FirmwareCheck::BadLength;
    }
    if (static_cast<std::uint16_t>(version_word >> 16U) != board_id) {
        return FirmwareCheck::BadBoardId;
    }
    const std::uint32_t len1 = at + offsetof(core::AppDescriptor, image_crc1);
    if (detail::crc_range(p, 0, len1) != crc1 ||
        detail::crc_range(p, at + offsetof(core::AppDescriptor, version_major), size) != crc2) {
        return FirmwareCheck::BadCrc;
    }
    return FirmwareCheck::Ok;
}

}  // namespace vk::bootloader
