#include <vk/local_display/pages.hpp>

#include <vk/estimation/math/quaternion.hpp>
#include <vk/local_display/font.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include <string_view>

namespace vk::local_display {

namespace {

// A line of text being built, cut off at the screen's width.
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

    // One decimal, rounded; "---" for a value that is not finite.
    Line& add_tenths(float value)
    {
        if (!std::isfinite(value)) {
            return add("---");
        }
        const long tenths = std::lround(static_cast<double>(value) * 10.0);
        const unsigned long magnitude = tenths < 0 ? static_cast<unsigned long>(-tenths) : static_cast<unsigned long>(tenths);
        if (tenths < 0) {
            add("-");
        }
        add(static_cast<std::uint32_t>(magnitude / 10)).add(".");
        return add(static_cast<std::uint32_t>(magnitude % 10));
    }

    // Pads with spaces to `column`.
    Line& to(std::size_t column)
    {
        while (size_ < column && size_ < text_.size()) {
            text_.at(size_++) = ' ';
        }
        return *this;
    }

    [[nodiscard]] std::string_view view() const { return {text_.data(), size_}; }

private:
    std::array<char, TextScreen::columns> text_{};
    std::size_t size_ = 0;
};

// Names short enough for their line, matching the commander's enums
// (vk/core/vehicle_control.hpp) and vehicle_status.
std::string_view mode_name(std::uint8_t mode)
{
    switch (mode) {
    case 0:
        return "STANDBY";
    case 1:
        return "TEST";
    default:
        return "MODE?";
    }
}

std::string_view refusal_name(std::uint8_t reason)
{
    static constexpr std::array<std::string_view, 9> names{
        "-", "BOARD", "ESTOP", "WRONG_MODE", "NO_CONTROL", "SETPOINTS", "BRINGUP", "ACT_POWER", "ACTUATORS"};
    return reason < names.size() ? names.at(reason) : "?";
}

std::string_view disarm_name(std::uint8_t reason)
{
    static constexpr std::array<std::string_view, 7> names{
        "-", "REQUEST", "ESTOP", "POWER_LOST", "TIMEOUT", "IDLE", "BRINGUP_ABORT"};
    return reason < names.size() ? names.at(reason) : "?";
}

std::string_view power_name(std::uint8_t state)
{
    static constexpr std::array<std::string_view, 4> names{"N/A", "OFF", "WAIT", "ON"};
    return state < names.size() ? names.at(state) : "?";
}

void header(TextScreen& screen, std::string_view title, Page page)
{
    screen.put(0, 0, title);
    Line number;
    number.add(static_cast<std::uint32_t>(page) + 1).add("/").add(static_cast<std::uint32_t>(page_count));
    screen.put_right(0, number.view());
}

void render_status(const Inputs& inputs, TextScreen& screen)
{
    header(screen, "STATUS", Page::Status);
    if (!inputs.status) {
        screen.put(1, 0, "NO STATUS");
        return;
    }
    const msg::VehicleStatus& s = *inputs.status;
    screen.put(1, 0, Line().add(mode_name(s.mode)).add(" ").add(s.armed ? "ARMED" : "DISARMED").view());
    screen.put(2, 0, Line().add("ESTOP ").add(s.emergency_stop ? "ON" : "OFF").view());
    screen.put(2, 10, Line().add("BOARD ").add(s.board_verified ? "OK" : "FAIL").view());
    screen.put(3, 0, Line().add("POWER ").add(power_name(s.actuator_power)).view());
    Line skipped;
    skipped.add("SKIP ");
    if (s.skipped_checks == 0) {
        skipped.add("NONE");
    } else if (s.skipped_checks == 1U) {
        skipped.add("ACT_POWER");  // bit 0 of COM_ARM_CHECK
    } else {
        skipped.add("MASK ").add(s.skipped_checks);
    }
    screen.put(4, 0, skipped.view());
    screen.put(5, 0, Line().add("REFUSED").to(8).add(refusal_name(s.arming_refusal)).view());
    screen.put(6, 0, Line().add("DISARM").to(8).add(disarm_name(s.disarm_reason)).view());
    screen.put(7, 0, Line().add("WATCHDOG").to(9).add(s.output_watchdog_trips).view());
}

void render_attitude(const Inputs& inputs, TextScreen& screen)
{
    header(screen, "ATTITUDE", Page::Attitude);
    if (!inputs.attitude) {
        screen.put(2, 0, "NO ESTIMATE");
        return;
    }
    const auto euler = estimation::Quaternion::from(inputs.attitude->q).euler();
    constexpr float degrees = 180.0F / std::numbers::pi_v<float>;
    const std::array<std::string_view, 3> names{"ROLL", "PITCH", "YAW"};
    const std::array<float, 3> angles{euler.roll, euler.pitch, euler.yaw};
    for (std::size_t axis = 0; axis < angles.size(); ++axis) {
        Line value;
        value.add_tenths(angles.at(axis) * degrees).add(" DEG");
        Line line;
        line.add(names.at(axis)).to(TextScreen::columns - value.view().size()).add(value.view());
        screen.put(2 + axis, 0, line.view());
    }
    screen.put(6, 0, Line().add("ACC REJECTED ").add(inputs.attitude->accel_rejected).view());
}

void render_link(const Inputs& inputs, TextScreen& screen)
{
    header(screen, "LINK", Page::Link);
    for (std::size_t instance = 0; instance < inputs.links.size(); ++instance) {
        const std::size_t first = 1 + (instance * 3);
        Line title;
        title.add("LINK").add(static_cast<std::uint32_t>(instance)).add(" ");
        const auto& link = inputs.links.at(instance);
        if (!link) {
            screen.put(first, 0, title.add("NONE").view());
            continue;
        }
        static constexpr std::array<std::string_view, 3> states{"WAITING", "CONNECTED", "SILENT"};
        title.add(link->handshake_state < states.size() ? states.at(link->handshake_state) : "?");
        screen.put(first, 0, title.view());
        screen.put_right(first, Line().add(static_cast<std::uint32_t>(link->rate_scale)).add("%").view());
        screen.put(first + 1, 0, Line().add(" DROPS ").add(link->drops).view());
        screen.put(first + 2, 0, Line().add(" LOST ").add(link->lost).view());
    }
}

}  // namespace

void TextScreen::clear()
{
    for (auto& line : text_) {
        line.fill(' ');
    }
}

void TextScreen::put(std::size_t line, std::size_t column, std::string_view text)
{
    if (line >= lines) {
        return;
    }
    for (const char c : text) {
        if (column >= columns) {
            return;
        }
        text_.at(line).at(column++) = c;
    }
}

void TextScreen::put_right(std::size_t line, std::string_view text)
{
    const std::size_t width = text.size() < columns ? text.size() : columns;
    put(line, columns - width, text.substr(text.size() - width));
}

std::string_view TextScreen::line(std::size_t line) const
{
    return {text_.at(line).data(), columns};
}

void TextScreen::draw(drivers::MonoFrame& frame) const
{
    // A cell is as tall as a frame page and a glyph column's bits are its rows,
    // as a frame byte's are, so each glyph column is one byte of its line's page.
    static_assert(cell_height == 8 && lines == drivers::MonoFrame::pages);
    static_assert(1 + (columns * cell_width) <= drivers::MonoFrame::width);
    frame.clear();
    for (std::size_t line = 0; line < lines; ++line) {
        auto& page = frame.bytes.at(line);
        for (std::size_t column = 0; column < columns; ++column) {
            const Glyph& g = glyph(text_.at(line).at(column));
            const std::size_t x0 = 1 + (column * cell_width);
            for (std::size_t x = 0; x < font_width; ++x) {
                page.at(x0 + x) = g.at(x);
            }
        }
    }
}

Page next_page(Page page)
{
    return static_cast<Page>((static_cast<std::size_t>(page) + 1) % page_count);
}

void render(Page page, const Inputs& inputs, TextScreen& screen)
{
    screen.clear();
    switch (page) {
    case Page::Status:
        render_status(inputs, screen);
        break;
    case Page::Attitude:
        render_attitude(inputs, screen);
        break;
    case Page::Link:
        render_link(inputs, screen);
        break;
    }
}

}  // namespace vk::local_display
