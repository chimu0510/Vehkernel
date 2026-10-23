// SPDX-License-Identifier: GPL-3.0-or-later
// Software SPI display bus on four GPIO_OUTPUT pins (ADR-0009, ADR-0022).
//
// A write-only bus towards a display controller such as the C50C's SSD1306:
// clock and data in SPI mode 0, most significant bit first, a reset line held
// low while active and a data/command line, low for commands. There is no
// chip select and nothing comes back. The CPU busy-waits through every bit,
// so a write blocks its work queue for its whole duration; the SSD1306
// driver bounds that by writing at most two pages per run.
#pragma once

#include <vk/hal/display_bus.hpp>
#include <vk/hal/gpio.hpp>
#include <vk/platform/cycle_counter.hpp>
#include <vk/platform/gpio.hpp>

#include <board.hpp>

#include <cstdint>
#include <span>

namespace vk::platform {

// The SSD1306's serial clock runs at up to 10 MHz: at least 50 ns low and
// 50 ns high. Every half period lasts at least twice that.
inline constexpr std::uint32_t soft_spi_min_half_period_ns = 100;

// Cycle counter ticks for a half period of at least `min_ns` at `sysclk_hz`.
constexpr std::uint32_t soft_spi_half_period_cycles(std::uint32_t sysclk_hz, std::uint32_t min_ns)
{
    const std::uint64_t ns_cycles = static_cast<std::uint64_t>(sysclk_hz) * min_ns;
    return static_cast<std::uint32_t>((ns_cycles + 999'999'999U) / 1'000'000'000U);
}

static_assert(soft_spi_half_period_cycles(168'000'000, soft_spi_min_half_period_ns) == 17,
              "17 cycles at 168 MHz last 101 ns, the shortest half period at or above 100 ns");

template <hal::GpioOutputPin SclPin, hal::GpioOutputPin SdaPin, hal::GpioOutputPin RstPin, hal::GpioOutputPin DcPin>
class SoftSpiDisplay {
public:
    static constexpr std::uint32_t half_period_cycles =
        soft_spi_half_period_cycles(vk::board::clock::sysclk_hz, soft_spi_min_half_period_ns);
    static_assert(static_cast<std::uint64_t>(half_period_cycles) * 1'000'000'000U >=
                      static_cast<std::uint64_t>(vk::board::clock::sysclk_hz) * soft_spi_min_half_period_ns,
                  "a software SPI half period must last at least 100 ns");

    // Drives every line, clock idle low and the controller out of reset; the
    // driver resets it before its first write.
    void init()
    {
        Scl::init(false);
        Sda::init(false);
        Dc::init(false);
        Rst::init(true);
    }

    void set_reset(bool active) { Rst::write(!active); }
    void write_commands(std::span<const std::uint8_t> bytes) { write(false, bytes); }
    void write_data(std::span<const std::uint8_t> bytes) { write(true, bytes); }

private:
    using Scl = PushPull<SclPin>;
    using Sda = PushPull<SdaPin>;
    using Rst = PushPull<RstPin>;
    using Dc = PushPull<DcPin>;

    static void half_period() { delay_cycles(half_period_cycles); }

    // The controller samples data on the rising edge, and the data/command
    // line with the last bit of each byte.
    static void write(bool data, std::span<const std::uint8_t> bytes)
    {
        Dc::write(data);
        for (const std::uint8_t byte : bytes) {
            for (int i = 7; i >= 0; --i) {
                Sda::write(((byte >> i) & 1U) != 0U);
                half_period();
                Scl::write(true);
                half_period();
                Scl::write(false);
            }
        }
        half_period();
    }
};

namespace detail {
template <class List>
struct SoftSpiDisplayOnPins;
template <class Scl, class Sda, class Rst, class Dc>
struct SoftSpiDisplayOnPins<hal::PinList<Scl, Sda, Rst, Dc>> {
    using type = SoftSpiDisplay<Scl, Sda, Rst, Dc>;
};
}  // namespace detail

// The software SPI bus of a display board device, from its pin list (clock,
// data, reset, data/command), e.g. SoftSpiDisplayOn<vk::board::devices::DISPLAY0::pins>.
template <class Pins>
using SoftSpiDisplayOn = typename detail::SoftSpiDisplayOnPins<Pins>::type;

}  // namespace vk::platform
