// SPDX-License-Identifier: GPL-3.0-or-later
// Example 01-led: blinks the on-board RGB LED with GPIO outputs and the system clock.
// ADR-0009 (layered HAL), ADR-0025 (board examples).
// Lesson: examples/c50c-v1_1/01-led/README.md
#include <board.hpp>
#include <support.hpp>
#include <vk/hal/clock.hpp>
#include <vk/platform/gpio.hpp>

#include <cstdint>

namespace {

// The board definition gives LIGHT_R, LIGHT_G and LIGHT_B timer functions for
// a PWM driver that does not exist yet. Drive them as GPIO outputs directly.
using LedRedPin = vk::hal::GpioOutput<vk::board::pins::LIGHT_R::port, vk::board::pins::LIGHT_R::number>;
using LedGreenPin = vk::hal::GpioOutput<vk::board::pins::LIGHT_G::port, vk::board::pins::LIGHT_G::number>;
using LedBluePin = vk::hal::GpioOutput<vk::board::pins::LIGHT_B::port, vk::board::pins::LIGHT_B::number>;

using LedRed = vk::platform::PushPull<LedRedPin>;
using LedGreen = vk::platform::PushPull<LedGreenPin>;
using LedBlue = vk::platform::PushPull<LedBluePin>;

void delay_ms(std::uint32_t ms)
{
    const std::uint64_t start = vk::hal::SystemClock::now_us();
    const std::uint64_t duration_us = static_cast<std::uint64_t>(ms) * 1000;
    while (vk::hal::SystemClock::now_us() - start < duration_us) {
    }
}

void set_color(bool r, bool g, bool b)
{
    LedRed::write(r);
    LedGreen::write(g);
    LedBlue::write(b);
}

}  // namespace

int main()
{
    example::init();

    LedRed::init(false);
    LedGreen::init(false);
    LedBlue::init(false);

    while (true) {
        set_color(true, false, false);
        delay_ms(500);

        set_color(false, true, false);
        delay_ms(500);

        set_color(false, false, true);
        delay_ms(500);

        set_color(false, false, false);
        delay_ms(500);
    }
}
