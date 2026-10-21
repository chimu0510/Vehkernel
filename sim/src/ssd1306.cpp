#include <vk/sim/ssd1306.hpp>

#include <cstdio>
#include <string>

namespace vk::sim {

namespace {

// How many argument bytes follow each multi-byte command (SSD1306 datasheet, section 9).
std::uint8_t argument_count(std::uint8_t command)
{
    switch (command) {
    case 0x20:  // memory addressing mode
    case 0x81:  // contrast
    case 0x8D:  // charge pump
    case 0xA8:  // multiplex ratio
    case 0xD3:  // display offset
    case 0xD5:  // clock divide ratio and oscillator frequency
    case 0xD9:  // pre-charge period
    case 0xDA:  // COM pins hardware configuration
    case 0xDB:  // VCOMH deselect level
        return 1;
    case 0x21:  // column address, horizontal and vertical addressing
    case 0x22:  // page address, horizontal and vertical addressing
        return 2;
    case 0x26:  // horizontal scroll setup
    case 0x27:
        return 6;
    case 0x29:  // vertical and horizontal scroll setup
    case 0x2A:
        return 5;
    case 0xA3:  // vertical scroll area
        return 2;
    default:
        return 0;
    }
}

}  // namespace

void Ssd1306::set_reset(bool active)
{
    if (active) {
        // A reset returns the controller to its power-on state; the RAM keeps
        // whatever it held, as on the chip.
        in_reset_ = true;
        display_on_ = false;
        charge_pump_ = false;
        segment_remap_ = false;
        com_reversed_ = false;
        inverted_ = false;
        all_on_ = false;
        page_ = 0;
        column_ = 0;
        pending_arguments_ = 0;
        commands_.clear();
        data_bytes_ = 0;
        return;
    }
    in_reset_ = false;
}

void Ssd1306::write_commands(std::span<const std::uint8_t> bytes)
{
    if (in_reset_) {
        return;
    }
    for (const std::uint8_t byte : bytes) {
        commands_.push_back(byte);
        if (pending_arguments_ > 0) {
            --pending_arguments_;
            if (pending_command_ == 0x8D) {
                charge_pump_ = (byte & 0x04U) != 0;
            }
            continue;
        }
        command(byte);
    }
}

void Ssd1306::command(std::uint8_t byte)
{
    if (byte <= 0x0F) {
        column_ = static_cast<std::uint8_t>((column_ & 0xF0U) | byte);
    } else if (byte <= 0x1F) {
        column_ = static_cast<std::uint8_t>((column_ & 0x0FU) | ((byte & 0x0FU) << 4U));
    } else if (byte >= 0xB0 && byte <= 0xB7) {
        page_ = static_cast<std::uint8_t>(byte & 0x07U);
    } else if (byte == 0xA0 || byte == 0xA1) {
        segment_remap_ = byte == 0xA1;
    } else if (byte == 0xC0 || byte == 0xC8) {
        com_reversed_ = byte == 0xC8;
    } else if (byte == 0xA4 || byte == 0xA5) {
        all_on_ = byte == 0xA5;
    } else if (byte == 0xA6 || byte == 0xA7) {
        inverted_ = byte == 0xA7;
    } else if (byte == 0xAE || byte == 0xAF) {
        display_on_ = byte == 0xAF;
    } else {
        pending_command_ = byte;
        pending_arguments_ = argument_count(byte);
    }
}

void Ssd1306::write_data(std::span<const std::uint8_t> bytes)
{
    if (in_reset_) {
        return;
    }
    for (const std::uint8_t byte : bytes) {
        // Page addressing: the column advances and stays within the page.
        if (column_ < width) {
            ram_.at(page_).at(column_) = byte;
            ++column_;
        }
        written_pages_ = static_cast<std::uint8_t>(written_pages_ | (1U << page_));
        ++data_bytes_;
    }
    write_image();
}

bool Ssd1306::displaying() const
{
    return !in_reset_ && display_on_ && charge_pump_;
}

bool Ssd1306::pixel(std::size_t x, std::size_t y) const
{
    if (!displaying() || x >= width || y >= height) {
        return false;
    }
    if (all_on_) {
        return true;
    }
    // The panel sits turned half a turn against its scan: image x is
    // 127 - SEG and image y is 63 - COM.
    const std::size_t segment = width - 1 - x;
    const std::size_t com = height - 1 - y;
    const std::size_t column = segment_remap_ ? width - 1 - segment : segment;
    const std::size_t row = com_reversed_ ? height - 1 - com : com;
    const bool lit = ((ram_.at(row / 8).at(column) >> (row % 8)) & 1U) != 0;
    return lit != inverted_;
}

std::uint8_t Ssd1306::ram(std::size_t page, std::size_t column) const
{
    return ram_.at(page).at(column);
}

std::uint8_t Ssd1306::take_written_pages()
{
    const std::uint8_t pages_written = written_pages_;
    written_pages_ = 0;
    return pages_written;
}

std::string Ssd1306::pbm() const
{
    std::string out = "P4\n" + std::to_string(width) + " " + std::to_string(height) + "\n";
    for (std::size_t y = 0; y < height; ++y) {
        for (std::size_t byte = 0; byte < width / 8; ++byte) {
            unsigned bits = 0;
            for (std::size_t bit = 0; bit < 8; ++bit) {
                bits = (bits << 1U) | (pixel((byte * 8) + bit, y) ? 1U : 0U);
            }
            out.push_back(static_cast<char>(bits));
        }
    }
    return out;
}

void Ssd1306::write_image() const
{
    if (image_path_.empty()) {
        return;
    }
    const std::string temporary = image_path_ + ".tmp";
    std::FILE* file = std::fopen(temporary.c_str(), "wb");
    if (file == nullptr) {
        return;
    }
    const std::string image = pbm();
    const bool written = std::fwrite(image.data(), 1, image.size(), file) == image.size();
    if (std::fclose(file) == 0 && written) {
        std::rename(temporary.c_str(), image_path_.c_str());
    }
}

}  // namespace vk::sim
