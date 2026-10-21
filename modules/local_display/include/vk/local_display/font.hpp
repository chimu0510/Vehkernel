// The local display's font (ADR-0022): printable ASCII, 5 x 7 pixels in a
// 6 x 8 cell, so 21 columns by 8 lines fit the 128 x 64 display.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace vk::local_display {

inline constexpr std::size_t font_width = 5;
inline constexpr std::size_t font_height = 7;
inline constexpr std::size_t cell_width = 6;
inline constexpr std::size_t cell_height = 8;
inline constexpr unsigned first_glyph = 0x20;
inline constexpr std::size_t glyph_count = 95;

// One glyph's columns, left first; bit n is row n from the top.
using Glyph = std::array<std::uint8_t, font_width>;

// The glyph of a printable ASCII character; '?' for any other.
const Glyph& glyph(char c);

}  // namespace vk::local_display
