// SPDX-License-Identifier: GPL-3.0-or-later
// Fixed-size line formatting without heap allocation (ADR-0007, ADR-0025).
#pragma once

#include <vk/local_display/pages.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace example {

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

    // Fixed-point float formatter: a sign (+/-), integer part, and two decimal places (e.g. "+0.00", "-9.81").
    Line& add_fixed(float value)
    {
        if (!std::isfinite(value)) {
            return add("---");
        }
        const long hundredths = std::lround(static_cast<double>(value) * 100.0);
        const unsigned long magnitude =
            hundredths < 0 ? static_cast<unsigned long>(-hundredths) : static_cast<unsigned long>(hundredths);
        if (hundredths < 0) {
            add("-");
        } else {
            add("+");
        }
        add(static_cast<std::uint32_t>(magnitude / 100)).add(".");
        const auto decimals = static_cast<std::uint32_t>(magnitude % 100);
        if (decimals < 10) {
            add("0");
        }
        return add(decimals);
    }

    // Fixed-point float formatter without positive sign: an optional minus sign, integer part, and two decimal places (e.g. "0.00", "2.50").
    Line& add_float(float value)
    {
        if (!std::isfinite(value)) {
            return add("---");
        }
        const long hundredths = std::lround(static_cast<double>(value) * 100.0);
        const unsigned long magnitude =
            hundredths < 0 ? static_cast<unsigned long>(-hundredths) : static_cast<unsigned long>(hundredths);
        if (hundredths < 0) {
            add("-");
        }
        add(static_cast<std::uint32_t>(magnitude / 100)).add(".");
        const auto decimals = static_cast<std::uint32_t>(magnitude % 100);
        if (decimals < 10) {
            add("0");
        }
        return add(decimals);
    }

    [[nodiscard]] std::string_view view() const { return {text_.data(), size_}; }

private:
    std::array<char, vk::local_display::TextScreen::columns> text_{};
    std::size_t size_ = 0;
};

}  // namespace example
