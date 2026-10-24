// SPDX-License-Identifier: GPL-3.0-or-later
// The STM32F4 bootloader's FIRMWARE region writes (ADR-0020, ADR-0023),
// against a fake Flash controller holding the whole of the C50C V1.1's
// 512 KB, laid out as the board header the board generator emits (its golden
// copy): an update erases the FIRMWARE region's sectors only, so the
// bootloader, the stored parameters and the event log survive it.
#include <board.hpp>
#include <vk/platform/firmware_flash.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <vector>

namespace {

namespace flash = vk::board::flash;

// The STM32F407's sectors 0 to 7 (RM0090 table 5): 4 x 16 KB, 64 KB, 3 x 128 KB.
constexpr std::uint32_t flash_base = 0x08000000;
constexpr std::array<std::uint32_t, 8> sector_sizes{16384, 16384, 16384, 16384, 65536, 131072, 131072, 131072};

constexpr std::uint32_t sector_start(unsigned sector)
{
    std::uint32_t address = flash_base;
    for (unsigned i = 0; i < sector; ++i) {
        address += sector_sizes.at(i);
    }
    return address;
}

constexpr std::uint32_t flash_end = sector_start(sector_sizes.size());

// The whole chip, word by word, programmed as NOR Flash is: erasing sets a
// sector to all ones, programming only clears bits.
struct FakeController {
    static inline std::vector<std::uint32_t> words;
    static inline std::vector<unsigned> erased;
    static inline bool fail_erase = false;
    static inline unsigned programmed = 0;

    static void reset(std::uint32_t fill)
    {
        words.assign((flash_end - flash_base) / 4, fill);
        erased.clear();
        fail_erase = false;
        programmed = 0;
    }

    static bool erase_sector(std::uint8_t sector)
    {
        if (fail_erase || sector >= sector_sizes.size()) {
            return false;
        }
        erased.push_back(sector);
        const auto first = (sector_start(sector) - flash_base) / 4;
        std::fill_n(words.begin() + first, sector_sizes.at(sector) / 4, 0xFFFFFFFFU);
        return true;
    }

    static vk::hal::FlashResult program_word(std::uint32_t address, std::uint32_t word)
    {
        if (address < flash_base || address >= flash_end || address % 4 != 0) {
            return vk::hal::FlashResult::Error;
        }
        ++programmed;
        words.at((address - flash_base) / 4) &= word;
        return vk::hal::FlashResult::Ok;
    }

    static std::uint32_t read_word(std::uint32_t address) { return words.at((address - flash_base) / 4); }
};

using Firmware = vk::platform::FirmwareFlash<flash::FIRMWARE, FakeController>;

bool region_holds(std::uint32_t address, std::uint32_t size, std::uint32_t value)
{
    const auto first = FakeController::words.begin() + (address - flash_base) / 4;
    return std::all_of(first, first + size / 4, [value](std::uint32_t w) { return w == value; });
}

class FirmwareFlashTest : public ::testing::Test {
protected:
    void SetUp() override { FakeController::reset(0x5A5A5A5AU); }
};

// The board header's layout agrees with the MCU's sectors.
static_assert(flash::FIRMWARE::address == sector_start(flash::FIRMWARE::first_sector));
static_assert(flash::FIRMWARE::address + flash::FIRMWARE::size == sector_start(flash::FIRMWARE::last_sector + 1));

TEST_F(FirmwareFlashTest, EraseCoversTheFirmwareSectorsOnly)
{
    ASSERT_TRUE(Firmware::erase());

    EXPECT_EQ(FakeController::erased, (std::vector<unsigned>{4, 5, 6, 7}));
    EXPECT_TRUE(region_holds(flash::FIRMWARE::address, flash::FIRMWARE::size, 0xFFFFFFFFU));
}

TEST_F(FirmwareFlashTest, EraseLeavesTheBootloaderParametersAndEventLog)
{
    ASSERT_TRUE(Firmware::erase());

    EXPECT_TRUE(region_holds(flash::BOOT::address, flash::BOOT::size, 0x5A5A5A5AU));
    EXPECT_TRUE(region_holds(flash::PARAMS::address, flash::PARAMS::size, 0x5A5A5A5AU));
    EXPECT_TRUE(region_holds(flash::EVENTLOG::address, flash::EVENTLOG::size, 0x5A5A5A5AU));
}

TEST_F(FirmwareFlashTest, EraseStopsAtAFailedSector)
{
    FakeController::fail_erase = true;
    EXPECT_FALSE(Firmware::erase());
}

TEST_F(FirmwareFlashTest, WritesCountFromTheRegionStart)
{
    FakeController::reset(0xFFFFFFFFU);
    const std::array<std::uint32_t, 2> words{0x20020000U, 0x08010201U};

    ASSERT_TRUE(Firmware::write(8, words));

    EXPECT_EQ(FakeController::read_word(flash::FIRMWARE::address + 8), 0x20020000U);
    EXPECT_EQ(Firmware::read(12), 0x08010201U);
}

TEST_F(FirmwareFlashTest, WritesOutsideTheRegionAreRefused)
{
    FakeController::reset(0xFFFFFFFFU);
    const std::array<std::uint32_t, 2> words{0, 0};

    EXPECT_FALSE(Firmware::write(flash::FIRMWARE::size - 4, words));  // runs past the end
    EXPECT_FALSE(Firmware::write(flash::FIRMWARE::size, words));
    EXPECT_FALSE(Firmware::write(0xFFFFFFFCU, words));  // would wrap around
    EXPECT_FALSE(Firmware::write(2, words));             // unaligned
    EXPECT_EQ(FakeController::programmed, 0U);
    EXPECT_TRUE(region_holds(flash_base, flash_end - flash_base, 0xFFFFFFFFU));
}

TEST_F(FirmwareFlashTest, ErasedWordsAreNotProgrammedAgain)
{
    FakeController::reset(0xFFFFFFFFU);
    const std::array<std::uint32_t, 3> words{0xFFFFFFFFU, 0x12345678U, 0xFFFFFFFFU};

    ASSERT_TRUE(Firmware::write(0, words));

    EXPECT_EQ(FakeController::programmed, 1U);
}

TEST_F(FirmwareFlashTest, ReadsOutsideTheRegionAreErased)
{
    EXPECT_EQ(Firmware::read(flash::FIRMWARE::size), 0xFFFFFFFFU);
    EXPECT_EQ(Firmware::read(1), 0xFFFFFFFFU);
}

}  // namespace
