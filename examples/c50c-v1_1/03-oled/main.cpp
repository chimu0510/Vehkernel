// SPDX-License-Identifier: GPL-3.0-or-later
// Example 03-oled: draws a title and a counter on the OLED through software SPI and the SSD1306 driver.
// ADR-0009 (layered HAL), ADR-0022 (local display and local menu), ADR-0025 (board examples).
// Lesson: examples/c50c-v1_1/03-oled/README.md
#include <support.hpp>
#include <vk/drivers/ssd1306.hpp>
#include <vk/hal/clock.hpp>
#include <vk/local_display/pages.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace {

// Formats text and numbers into a fixed-size line buffer without heap allocation (ADR-0007).
class Line {
public:
    Line& add(std::string_view text)
    {
        for (const char c : text) {
            if (size_ < text_.size()) {
                text_.at(size_++) = c;
            }
        }
        return *this;
    }

    Line& add(std::uint32_t value)
    {
        std::array<char, 10> digits{};
        std::size_t count = 0;
        do {
            digits.at(count++) = static_cast<char>('0' + (value % 10));
            value /= 10;
        } while (value != 0);
        while (count > 0) {
            add(std::string_view(&digits.at(--count), 1));
        }
        return *this;
    }

    [[nodiscard]] std::string_view view() const { return {text_.data(), size_}; }

private:
    std::array<char, vk::local_display::TextScreen::columns> text_{};
    std::size_t size_ = 0;
};

}  // namespace

int main(int argc, char** argv)
{
    example::init(argc, argv);

    auto& display = example::display();

    constexpr std::uint64_t step_us = 100'000;  // 10 Hz
    std::uint64_t next_us = vk::hal::SystemClock::now_us();

    std::uint32_t counter = 0;
    vk::local_display::TextScreen screen;

    while (example::running()) {
        screen.clear();
        screen.put(0, 0, "VEHKERNEL 03-OLED");
        screen.put(2, 0, Line().add("COUNTER: ").add(counter).view());
        screen.draw(display.frame());

        static_cast<void>(display.update(vk::hal::SystemClock::now_us(), vk::drivers::MonoFrame::pages));
        example::report("counter", counter);
        ++counter;

        next_us += step_us;
        example::sleep_until_us(next_us);
    }

    return 0;
}
