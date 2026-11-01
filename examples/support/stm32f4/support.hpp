// SPDX-License-Identifier: GPL-3.0-or-later
// Support code for STM32F4 board examples (spec 0013, ADR-0025).
#pragma once

#include <board.hpp>
#include <vk/core/static_instance.hpp>
#include <vk/drivers/ssd1306.hpp>
#include <vk/hal/clock.hpp>
#include <vk/hal/display_bus.hpp>
#include <vk/platform/platform.hpp>
#include <vk/platform/soft_spi.hpp>

#include <cstdint>
#include <string_view>

namespace example {

namespace detail {

using Display = vk::board::devices::DISPLAY0;
static_assert(Display::type == "SSD1306" && Display::bus == "SOFT_SPI",
              "the C50C's display is an SSD1306 on software SPI");

using DisplayBus = vk::platform::SoftSpiDisplayOn<Display::pins>;
static_assert(vk::hal::DisplayBus<DisplayBus>);

inline DisplayBus display_bus;
inline vk::core::StaticInstance<vk::drivers::Ssd1306> display_driver;

}  // namespace detail

// Brings up the platform: configures fault handlers, interrupt priorities,
// the clock tree from the board definition, the microsecond clock and the
// cycle counter (ADR-0009).
inline void init(int /*argc*/ = 0, char** /*argv*/ = nullptr)
{
    vk::platform::init();
}

// Emits a key=value line in SITL for L3 testing (spec 0013); does nothing on
// the board.
template <typename T>
inline void report(std::string_view key, const T& value)
{
    static_cast<void>(key);
    static_cast<void>(value);
}

// Returns true on the board (runs indefinitely).
inline bool running()
{
    return true;
}

// Busy-waits until the system microsecond clock reaches target_us.
inline void sleep_until_us(std::uint64_t target_us)
{
    while (vk::hal::SystemClock::now_us() < target_us) {
    }
}

// Returns the board's SSD1306 OLED display driver on software SPI.
inline vk::drivers::Ssd1306& display()
{
    if (!detail::display_driver.has_value()) {
        detail::display_bus.init();
        detail::display_driver.emplace(vk::hal::DisplayDevice(detail::display_bus));
    }
    return *detail::display_driver;
}

}  // namespace example
