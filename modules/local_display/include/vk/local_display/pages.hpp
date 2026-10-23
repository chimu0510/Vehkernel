// SPDX-License-Identifier: GPL-3.0-or-later
// The local display's pages (ADR-0022): text rendered from the latest topics.
//
// Rendering uses neither the bus nor FreeRTOS, so it is tested on the
// development host; the LocalDisplay work item feeds it.
#pragma once

#include <vk/drivers/ssd1306.hpp>
#include <vk/msg/link_status.hpp>
#include <vk/msg/vehicle_attitude.hpp>
#include <vk/msg/vehicle_status.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>

namespace vk::local_display {

// 21 columns by 8 lines of text, the font's cells on a 128 x 64 display.
class TextScreen {
public:
    static constexpr std::size_t columns = 21;
    static constexpr std::size_t lines = 8;

    TextScreen() { clear(); }

    // Fills every cell with spaces.
    void clear();
    // Writes `text` from `column` of `line`; what does not fit is cut off.
    void put(std::size_t line, std::size_t column, std::string_view text);
    // Writes `text` ending at the last column of `line`.
    void put_right(std::size_t line, std::string_view text);
    [[nodiscard]] std::string_view line(std::size_t line) const;

    // Draws every cell into `frame`, one pixel in from the left edge.
    void draw(drivers::MonoFrame& frame) const;

    friend bool operator==(const TextScreen&, const TextScreen&) = default;

private:
    std::array<std::array<char, columns>, lines> text_{};
};

enum class Page : std::uint8_t {
    Status,
    Attitude,
    Link,
};
inline constexpr std::size_t page_count = 3;

// The next page in turn, as a short press of the user key moves.
Page next_page(Page page);

// The latest of each topic a page shows; nothing if never published.
struct Inputs {
    std::optional<msg::VehicleStatus> status;
    std::optional<msg::VehicleAttitude> attitude;
    std::array<std::optional<msg::LinkStatus>, msg::LinkStatus::instances> links;
};

void render(Page page, const Inputs& inputs, TextScreen& screen);

}  // namespace vk::local_display
