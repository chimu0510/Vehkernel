// The firmware image's app descriptor (ADR-0023), placed right after the
// vector table by the linker script. The build fills the CRCs, the size and
// the git hash after linking; the version and the board ID are known here.
#include <vk/core/app_descriptor.hpp>

#include <board.hpp>

#include <cstdint>

namespace {

[[gnu::section(".app_descriptor"), gnu::used]] const vk::core::AppDescriptor app_descriptor{
    .signature = vk::core::app_descriptor_signature,
    .image_crc1 = 0,
    .image_crc2 = 0,
    .image_size = 0,
    .git_hash = 0,
    .version_major = VK_VERSION_MAJOR,
    .version_minor = VK_VERSION_MINOR,
    .board_id = vk::board::board_id,
    .reserved = {},
};

}  // namespace
