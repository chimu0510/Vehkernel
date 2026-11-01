// SPDX-License-Identifier: GPL-3.0-or-later
// Example 02-user-key: cycles RGB LED colours on debounced presses of user key S3.
// ADR-0009 (layered HAL), ADR-0022 (local display and local menu), ADR-0025 (board examples).
// Lesson: examples/c50c-v1_1/02-user-key/README.md
#include <board.hpp>
#include <support.hpp>
#include <vk/hal/clock.hpp>
#include <vk/platform/gpio.hpp>

#include <array>
#include <cstddef>
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

// User key S3 as a GPIO input line with the board's pull resistor.
using UserKeyLine = vk::platform::InputLine<vk::board::user_key::pin>;

// Reads whether user key S3 is physically pressed right now, comparing the pin
// reading against the board definition's active level.
bool is_key_pressed()
{
    return UserKeyLine::is_high() == vk::board::user_key::pressed_high;
}

struct Color {
    bool r;
    bool g;
    bool b;
};

// Fixed list of colours: off, three primaries, and four secondary mixes.
constexpr std::array<Color, 8> colors{{
    {false, false, false},  // Off
    {true, false, false},   // Red
    {false, true, false},   // Green
    {false, false, true},   // Blue
    {true, true, false},    // Yellow (Red + Green)
    {false, true, true},    // Cyan (Green + Blue)
    {true, false, true},    // Magenta (Red + Blue)
    {true, true, true},     // White (Red + Green + Blue)
}};

void set_color(const Color& color)
{
    LedRed::write(color.r);
    LedGreen::write(color.g);
    LedBlue::write(color.b);
}

void delay_ms(std::uint32_t ms)
{
    const std::uint64_t start = vk::hal::SystemClock::now_us();
    const std::uint64_t duration_us = static_cast<std::uint64_t>(ms) * 1000;
    while (vk::hal::SystemClock::now_us() - start < duration_us) {
    }
}

// Plain debouncer: samples every 10 ms. A candidate level counts after
// debounce_count consecutive agreeing samples (30 ms). Transitions to the
// pressed state return true once; releases return false.
class Debouncer {
public:
    static constexpr unsigned debounce_count = 3;

    bool update(bool raw_pressed)
    {
        if (raw_pressed == candidate_) {
            if (agreeing_ < debounce_count) {
                ++agreeing_;
                if (agreeing_ == debounce_count && debounced_ != candidate_) {
                    debounced_ = candidate_;
                    return debounced_;
                }
            }
        } else {
            candidate_ = raw_pressed;
            agreeing_ = 1;
        }
        return false;
    }

private:
    bool candidate_ = false;
    bool debounced_ = false;
    unsigned agreeing_ = 0;
};

}  // namespace

int main()
{
    example::init();

    UserKeyLine::init();

    LedRed::init(false);
    LedGreen::init(false);
    LedBlue::init(false);

    std::size_t color_index = 0;
    set_color(colors[color_index]);

    Debouncer debouncer;

    while (true) {
        delay_ms(10);
        if (debouncer.update(is_key_pressed())) {
            color_index = (color_index + 1) % colors.size();
            set_color(colors[color_index]);
        }
    }
}
