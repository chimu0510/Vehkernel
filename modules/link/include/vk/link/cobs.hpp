// SPDX-License-Identifier: GPL-3.0-or-later
// Consistent Overhead Byte Stuffing (ADR-0002): removes every zero byte from
// a frame, so a zero can mark the end of each frame on the wire.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace vk::link {

// Largest COBS encoding of `size` bytes, without the zero delimiter.
constexpr std::size_t cobs_max_encoded_size(std::size_t size)
{
    return size + size / 254 + 1;
}

// Encodes `in` into `out`; returns the bytes written, 0 if `out` is too small.
std::size_t cobs_encode(std::span<const std::uint8_t> in, std::span<std::uint8_t> out);

// Decodes `in` (without the delimiter) into `out`; returns the bytes written,
// 0 if `in` is not valid COBS or `out` is too small.
std::size_t cobs_decode(std::span<const std::uint8_t> in, std::span<std::uint8_t> out);

}  // namespace vk::link
