// Simulated SSD1306 display controller (ADR-0018, ADR-0022).
//
// SITL's display, and the display of unit tests: it satisfies the display
// bus contract itself, decodes the command and data bytes the driver sends
// into the controller's 128 x 64 display RAM, and shows that RAM as the panel
// does. Page addressing is modelled, which is what the driver uses; the
// commands that only tune the panel's drive are accepted and their bytes
// kept, so tests can check the initialisation sequence.
//
// The image follows the vendor firmware's coordinates: x to the right and y
// down from the top left corner of the panel as it sits on the C50C. With the
// vendor's segment remap (0xA1) and normal COM scan (0xC0) that puts RAM row
// r at image row 63 - r, as the vendor's pixel mapping does.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace vk::sim {

class Ssd1306 {
public:
    static constexpr std::size_t width = 128;
    static constexpr std::size_t height = 64;
    static constexpr std::size_t pages = height / 8;

    // The display bus contract (vk/hal/display_bus.hpp).
    void set_reset(bool active);
    void write_commands(std::span<const std::uint8_t> bytes);
    void write_data(std::span<const std::uint8_t> bytes);

    // Whether the panel shows the RAM: out of reset, charge pump on and display on.
    [[nodiscard]] bool displaying() const;
    // A pixel of the image as the panel shows it; false while not displaying.
    [[nodiscard]] bool pixel(std::size_t x, std::size_t y) const;
    // One byte of display RAM.
    [[nodiscard]] std::uint8_t ram(std::size_t page, std::size_t column) const;

    // Every command byte since the last reset, arguments included.
    [[nodiscard]] const std::vector<std::uint8_t>& commands() const { return commands_; }
    // The RAM pages written since the last call, one bit per page.
    std::uint8_t take_written_pages();
    // Data bytes written since the last reset.
    [[nodiscard]] std::size_t data_bytes() const { return data_bytes_; }

    // The image as a binary PBM (P4) file.
    [[nodiscard]] std::string pbm() const;
    // Writes pbm() to `path` after every data write, replacing the file in
    // one rename so a reader never sees half an image; empty to stop.
    void write_image_to(std::string path) { image_path_ = std::move(path); }

private:
    void command(std::uint8_t byte);
    void write_image() const;

    std::array<std::array<std::uint8_t, width>, pages> ram_{};
    std::vector<std::uint8_t> commands_;
    bool in_reset_ = true;
    bool display_on_ = false;
    bool charge_pump_ = false;
    bool segment_remap_ = false;  // 0xA1: column 0 drives SEG127
    bool com_reversed_ = false;   // 0xC8: COM scan from COM63
    bool inverted_ = false;       // 0xA7
    bool all_on_ = false;         // 0xA5
    std::uint8_t page_ = 0;
    std::uint8_t column_ = 0;
    std::uint8_t pending_command_ = 0;  // a command still waiting for its arguments
    std::uint8_t pending_arguments_ = 0;
    std::uint8_t written_pages_ = 0;
    std::size_t data_bytes_ = 0;
    std::string image_path_;
};

}  // namespace vk::sim
