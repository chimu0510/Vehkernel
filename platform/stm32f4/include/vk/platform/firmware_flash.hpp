// SPDX-License-Identifier: GPL-3.0-or-later
// The FIRMWARE region of the STM32F4's internal Flash, as the Vehkernel
// bootloader writes it (ADR-0020, ADR-0023).
//
// `Region` is a Flash region of the board header, vk::board::flash::FIRMWARE
// on a board: sectors first_sector to last_sector, of any sizes. Offsets
// count from the region's start. Only the region's sectors are ever erased,
// so the BOOT, parameter and event log sectors survive a firmware update, and
// nothing is written outside it. `Controller` is the Flash controller:
// detail::flash_erase_sector() and detail::flash_program_word() on the board
// (InternalController), a fake in the unit tests, so this file needs no
// hardware.
#pragma once

#include <vk/hal/flash.hpp>

#include <cstdint>
#include <span>

namespace vk::platform {

template <class Region, class Controller>
class FirmwareFlash {
public:
    static_assert(Region::first_sector <= Region::last_sector, "the region needs a sector");

    [[nodiscard]] static constexpr std::uint32_t size() { return Region::size; }

    // Erases the region's sectors, first to last; false at the first that fails.
    static bool erase()
    {
        for (unsigned sector = Region::first_sector; sector <= Region::last_sector; ++sector) {
            if (!Controller::erase_sector(static_cast<std::uint8_t>(sector))) {
                return false;
            }
        }
        return true;
    }

    // Writes `words` from `offset`, a multiple of 4; false when any falls
    // outside the region or the controller reports an error. An erased word
    // asked to stay erased is left alone.
    static bool write(std::uint32_t offset, std::span<const std::uint32_t> words)
    {
        if (offset % 4 != 0 || offset > Region::size || words.size() > (Region::size - offset) / 4) {
            return false;
        }
        for (const std::uint32_t word : words) {
            if (word != 0xFFFFFFFFU || Controller::read_word(Region::address + offset) != 0xFFFFFFFFU) {
                if (Controller::program_word(Region::address + offset, word) != hal::FlashResult::Ok) {
                    return false;
                }
            }
            offset += 4;
        }
        return true;
    }

    // The word at `offset`; erased (all ones) outside the region.
    [[nodiscard]] static std::uint32_t read(std::uint32_t offset)
    {
        if (offset % 4 != 0 || offset >= Region::size) {
            return 0xFFFFFFFFU;
        }
        return Controller::read_word(Region::address + offset);
    }
};

}  // namespace vk::platform
