// The parameter region of the STM32F4's internal Flash (ADR-0020).
//
// `Region` is a Flash region of the board header, such as
// vk::board::flash::PARAMS: equal sectors from first_sector to last_sector.
// Only those sectors are ever erased or programmed. Erasing or programming
// stalls every Flash fetch while it runs, ISRs included; the parameter
// storage bounds how much it does per run (ADR-0020).
#pragma once

#include <vk/hal/flash.hpp>

#include <cstddef>
#include <cstdint>

namespace vk::platform {

namespace detail {
// The Flash controller, register by register; false or Error on a reported error.
bool flash_erase_sector(std::uint8_t sector);
hal::FlashResult flash_program_word(std::uint32_t address, std::uint32_t word);
}  // namespace detail

template <class Region>
class InternalFlash {
public:
    static constexpr std::size_t count = Region::last_sector - Region::first_sector + 1;
    static constexpr std::size_t size = Region::size / count;
    static_assert(count >= 2 && size * count == Region::size, "the region needs two or more equal sectors");

    [[nodiscard]] std::size_t sector_count() const { return count; }
    [[nodiscard]] std::size_t sector_size() const { return size; }

    bool erase(std::size_t sector)
    {
        return sector < count && detail::flash_erase_sector(static_cast<std::uint8_t>(Region::first_sector + sector));
    }

    hal::FlashResult program(std::size_t sector, std::size_t offset, std::uint32_t word)
    {
        if (sector >= count || offset >= size || offset % 4 != 0) {
            return hal::FlashResult::Error;
        }
        return detail::flash_program_word(address(sector, offset), word);
    }

    [[nodiscard]] std::uint32_t read(std::size_t sector, std::size_t offset) const
    {
        if (sector >= count || offset >= size || offset % 4 != 0) {
            return 0xFFFFFFFFU;
        }
        // Flash is memory-mapped; the data cache is reset after every erase.
        return *reinterpret_cast<const volatile std::uint32_t*>(address(sector, offset));  // NOLINT(performance-no-int-to-ptr)
    }

private:
    static constexpr std::uint32_t address(std::size_t sector, std::size_t offset)
    {
        return Region::address + static_cast<std::uint32_t>(sector * size + offset);
    }
};

}  // namespace vk::platform
