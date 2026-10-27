// SPDX-License-Identifier: GPL-3.0-or-later
// The app descriptor (ADR-0023): ArduPilot's unsigned layout from
// libraries/AP_CheckFirmware/AP_CheckFirmware.h, so ArduPilot's bootloader
// and the Vehkernel bootloader check the same record.
//
// A firmware image holds one, right after its vector table. The build fills
// the CRCs, the size and the git hash after linking
// (tools/firmware/finalize_image.py):
// - image_crc1: CRC-32 of the image from its start to image_crc1, signature
//   included;
// - image_crc2: CRC-32 from version_major to the end of the image;
// both with ArduPilot's crc32_small, polynomial 0xEDB88320 starting from 0
// with no final inversion.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace vk::core {

struct AppDescriptor {
    std::array<std::uint8_t, 8> signature;
    std::uint32_t image_crc1;
    std::uint32_t image_crc2;
    std::uint32_t image_size;  // bytes, the whole image
    std::uint32_t git_hash;    // first 32 bits of the commit
    std::uint8_t version_major;
    std::uint8_t version_minor;
    std::uint16_t board_id;
    std::array<std::uint8_t, 8> reserved;
};

// AP_APP_DESCRIPTOR_SIGNATURE_UNSIGNED.
inline constexpr std::array<std::uint8_t, 8> app_descriptor_signature{0x40, 0xa2, 0xe4, 0xf1,
                                                                        0x64, 0x68, 0x91, 0x06};

static_assert(sizeof(AppDescriptor) == 36, "APP_DESCRIPTOR_UNSIGNED_TOTAL_LENGTH");
static_assert(offsetof(AppDescriptor, image_crc1) == 8 && offsetof(AppDescriptor, version_major) == 24 &&
              offsetof(AppDescriptor, board_id) == 26);

// The firmware image's descriptor (defined in the platform).
extern const AppDescriptor app_descriptor;

// Reads fields from an AppDescriptor through volatile access so that the compiler
// or link-time optimization does not fold the compile-time zero initializers of
// fields that the build fills into the image after linking (finalize_image.py, ADR-0023).
template <typename T>
[[nodiscard]] inline T read_descriptor_field(const T& field) noexcept
{
    return *static_cast<const volatile T*>(&field);
}

}  // namespace vk::core
