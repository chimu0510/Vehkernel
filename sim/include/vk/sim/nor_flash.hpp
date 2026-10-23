// SPDX-License-Identifier: GPL-3.0-or-later
// Model of the parameter sectors of NOR Flash, as on the STM32F407 (ADR-0020).
//
// One model serves as the fake Flash of the parameter storage unit tests and
// as SITL's parameter Flash (ADR-0018). It keeps the rules of the hardware:
// an erase sets a whole sector to all ones, programming a word can only
// clear bits, and programming a word that is not erased is refused. Power
// can be lost at any operation: the operation is left half done (a word
// half programmed, a sector half erased) and every later operation fails
// until power_on(), which is a reboot. In SITL the contents may be kept in a
// file across runs.
#pragma once

#include <vk/hal/flash.hpp>

#include <array>
#include <cstddef>
#include <cstdint>

namespace vk::sim {

class NorFlash {
public:
    static constexpr std::size_t max_bytes = 2 * 16 * 1024;  // S1 and S2 of the F407

    // `sector_size` bytes per sector, a multiple of 4; `sector_count` *
    // `sector_size` must fit max_bytes. Blank at start.
    explicit NorFlash(std::size_t sector_count = 2, std::size_t sector_size = 16 * 1024);

    NorFlash(const NorFlash&) = delete;
    NorFlash& operator=(const NorFlash&) = delete;
    NorFlash(NorFlash&&) = delete;
    NorFlash& operator=(NorFlash&&) = delete;
    ~NorFlash();

    [[nodiscard]] std::size_t sector_count() const { return sector_count_; }
    [[nodiscard]] std::size_t sector_size() const { return sector_size_; }
    bool erase(std::size_t sector);
    hal::FlashResult program(std::size_t sector, std::size_t offset, std::uint32_t word);
    [[nodiscard]] std::uint32_t read(std::size_t sector, std::size_t offset) const;

    // Keeps the contents in `path` from now on: an existing file of the right
    // size is loaded, anything else is replaced by the blank contents. False
    // if the file cannot be opened or written.
    bool attach_file(const char* path);

    // Loses power during the operation after `operations` more erases and
    // programs succeed: 0 loses it during the next one.
    void lose_power_after(std::size_t operations) { power_loss_in_ = operations + 1; }
    // Power back: a reboot. Operations work again; the contents stay as left.
    void power_on()
    {
        powered_ = true;
        power_loss_in_ = 0;
    }
    [[nodiscard]] bool powered() const { return powered_; }

    // Erases and programs that succeeded, since construction.
    [[nodiscard]] std::size_t operations() const { return operations_; }

private:
    // Counts an operation; false if power is lost during it.
    bool survive();
    void save(std::size_t first_word, std::size_t words);

    std::size_t sector_count_;
    std::size_t sector_size_;
    std::array<std::uint32_t, max_bytes / 4> words_{};
    bool powered_ = true;
    std::size_t power_loss_in_ = 0;  // 0: never; else operations left, this one included
    std::size_t operations_ = 0;
    int fd_ = -1;
};

static_assert(hal::ParamFlash<NorFlash>);

}  // namespace vk::sim
