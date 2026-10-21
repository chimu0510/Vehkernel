#include <vk/drivers/ssd1306.hpp>

#include <array>
#include <cstdint>

namespace vk::drivers {

namespace {

// The vendor firmware's initialisation (WHEELTEC_C50X 2026.05.29, oled.c,
// OLED_Init), commands with their arguments: display off; clock 0x80; 64
// rows; no offset; start line 0; charge pump on; page addressing; segment
// remap; normal COM scan; alternative COM pins; contrast 0xEF; pre-charge
// 0xF1; VCOMH 0x30; follow RAM; not inverted; display on.
constexpr std::array<std::uint8_t, 25> init_sequence{
    0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0x8D, 0x14, 0x20, 0x02, 0xA1,
    0xC0, 0xDA, 0x12, 0x81, 0xEF, 0xD9, 0xF1, 0xDB, 0x30, 0xA4, 0xA6, 0xAF};

std::uint8_t reverse_bits(std::uint8_t byte)
{
    std::uint8_t out = 0;
    for (unsigned bit = 0; bit < 8; ++bit) {
        out = static_cast<std::uint8_t>((out << 1U) | ((byte >> bit) & 1U));
    }
    return out;
}

}  // namespace

void MonoFrame::set(std::size_t x, std::size_t y, bool lit)
{
    if (x >= width || y >= height) {
        return;
    }
    auto& byte = bytes.at(y / 8).at(x);
    const auto mask = static_cast<std::uint8_t>(1U << (y % 8));
    byte = static_cast<std::uint8_t>(lit ? (byte | mask) : (byte & ~mask));
}

bool MonoFrame::get(std::size_t x, std::size_t y) const
{
    if (x >= width || y >= height) {
        return false;
    }
    return ((bytes.at(y / 8).at(x) >> (y % 8)) & 1U) != 0;
}

std::size_t Ssd1306::update(std::uint64_t now_us, std::size_t max_pages)
{
    if (state_ == State::Idle) {
        device_.set_reset(true);
        reset_until_us_ = now_us + reset_time_us;
        state_ = State::Resetting;
        return 0;
    }
    if (state_ == State::Resetting) {
        if (now_us < reset_until_us_) {
            return 0;
        }
        device_.set_reset(false);
        initialise();
        state_ = State::Ready;
    }
    std::size_t written = 0;
    for (std::size_t scanned = 0; scanned < MonoFrame::pages && written < max_pages; ++scanned) {
        const std::size_t page = next_page_;
        next_page_ = (next_page_ + 1) % MonoFrame::pages;
        if (stale_.at(page) || frame_.bytes.at(page) != shown_.bytes.at(page)) {
            write_page(page);
            ++written;
        }
    }
    return written;
}

void Ssd1306::initialise()
{
    device_.write_commands(init_sequence);
    // The RAM holds whatever it held before the reset.
    stale_.fill(true);
    next_page_ = 0;
}

void Ssd1306::write_page(std::size_t page)
{
    // The vendor firmware's coordinates put frame row y at RAM row 63 - y:
    // frame page p is RAM page 7 - p with its bits reversed, in the same columns.
    const auto ram_page = static_cast<std::uint8_t>(MonoFrame::pages - 1 - page);
    const std::array<std::uint8_t, 3> address{static_cast<std::uint8_t>(0xB0U | ram_page), 0x00, 0x10};
    device_.write_commands(address);
    std::array<std::uint8_t, MonoFrame::width> data{};
    const auto& source = frame_.bytes.at(page);
    for (std::size_t column = 0; column < MonoFrame::width; ++column) {
        data.at(column) = reverse_bits(source.at(column));
    }
    device_.write_data(data);
    shown_.bytes.at(page) = source;
    stale_.at(page) = false;
}

}  // namespace vk::drivers
