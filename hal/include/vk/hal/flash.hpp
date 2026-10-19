// Parameter Flash contract (ADR-0009, ADR-0020).
//
// The parameter region of internal Flash is a few equal sectors, two on the
// STM32F407 (S1 and S2). A platform's class for it, real or simulated,
// satisfies ParamFlash: NOR Flash erases a whole sector to all ones, and
// programming a word can only clear bits, so a word must be erased before it
// is programmed. core/ reaches it only through FlashSectors, which binds any
// ParamFlash at start-up, so parameter storage is not a template.
#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>

namespace vk::hal {

enum class FlashResult : std::uint8_t {
    Ok = 0,
    NotErased = 1,  // the word was programmed before; nothing changed
    Error = 2,      // the controller reported an error, or the address is outside the region
};

template <class T>
concept ParamFlash = requires(T& flash, const T& const_flash, std::size_t sector, std::size_t offset,
                              std::uint32_t word) {
    { const_flash.sector_count() } -> std::same_as<std::size_t>;
    { const_flash.sector_size() } -> std::same_as<std::size_t>;  // bytes
    // Sets every word of the sector to all ones; false on an error. Stalls
    // every Flash fetch for up to seconds on hardware (ADR-0020).
    { flash.erase(sector) } -> std::same_as<bool>;
    // Programs the word at a byte offset, a multiple of 4, in the sector.
    { flash.program(sector, offset, word) } -> std::same_as<FlashResult>;
    { const_flash.read(sector, offset) } -> std::same_as<std::uint32_t>;
};

// The parameter Flash of the build.
class FlashSectors {
public:
    // FlashSectors meets ParamFlash itself; copies are copies, not wrappers.
    template <ParamFlash Flash>
        requires(!std::same_as<Flash, FlashSectors>)
    explicit FlashSectors(Flash& flash)
        : flash_(&flash), sector_count_(flash.sector_count()), sector_size_(flash.sector_size()),
          erase_(&erase_on<Flash>), program_(&program_on<Flash>), read_(&read_on<Flash>)
    {
    }

    [[nodiscard]] std::size_t sector_count() const { return sector_count_; }
    [[nodiscard]] std::size_t sector_size() const { return sector_size_; }
    [[nodiscard]] bool erase(std::size_t sector) const { return erase_(flash_, sector); }

    [[nodiscard]] FlashResult program(std::size_t sector, std::size_t offset, std::uint32_t word) const
    {
        return program_(flash_, sector, offset, word);
    }

    [[nodiscard]] std::uint32_t read(std::size_t sector, std::size_t offset) const
    {
        return read_(flash_, sector, offset);
    }

private:
    using EraseFn = bool (*)(void*, std::size_t);
    using ProgramFn = FlashResult (*)(void*, std::size_t, std::size_t, std::uint32_t);
    using ReadFn = std::uint32_t (*)(const void*, std::size_t, std::size_t);

    template <ParamFlash Flash>
    static bool erase_on(void* flash, std::size_t sector)
    {
        return static_cast<Flash*>(flash)->erase(sector);
    }

    template <ParamFlash Flash>
    static FlashResult program_on(void* flash, std::size_t sector, std::size_t offset, std::uint32_t word)
    {
        return static_cast<Flash*>(flash)->program(sector, offset, word);
    }

    template <ParamFlash Flash>
    static std::uint32_t read_on(const void* flash, std::size_t sector, std::size_t offset)
    {
        return static_cast<const Flash*>(flash)->read(sector, offset);
    }

    void* flash_;
    std::size_t sector_count_;
    std::size_t sector_size_;
    EraseFn erase_;
    ProgramFn program_;
    ReadFn read_;
};

}  // namespace vk::hal
