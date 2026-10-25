// SPDX-License-Identifier: GPL-3.0-or-later
// Fuzz target for the COBS decoder (ADR-0002, ADR-0018). The input is the
// bytes between two delimiters. Each buffer is a heap allocation of exactly
// its size, so AddressSanitizer catches a read or write one past its end.
//
// Checks, besides the sanitizers:
// - decoding never reports more bytes than the output holds;
// - whatever decodes encodes again to bytes that decode to the same;
// - any input, encoded, decodes back to itself, and its encoding holds no zero.
#include <vk/link/cobs.hpp>
#include <vk/link/frame.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <span>
#include <vector>

namespace {

void check(bool condition)
{
    if (!condition) {
        std::abort();
    }
}

std::vector<std::uint8_t> decode(std::span<const std::uint8_t> encoded, std::size_t capacity)
{
    std::vector<std::uint8_t> out(capacity);
    const std::size_t size = vk::link::cobs_decode(encoded, out);
    check(size <= capacity);
    out.resize(size);
    return out;
}

std::vector<std::uint8_t> encode(std::span<const std::uint8_t> plain)
{
    std::vector<std::uint8_t> out(vk::link::cobs_max_encoded_size(plain.size()));
    const std::size_t size = vk::link::cobs_encode(plain, out);
    check(size != 0 && size <= out.size());
    out.resize(size);
    check(std::ranges::find(out, std::uint8_t{0}) == out.end());
    return out;
}

}  // namespace

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size)
{
    const std::vector<std::uint8_t> input(data, data + size);

    // As the frame reader decodes, into a frame-sized buffer.
    const std::vector<std::uint8_t> decoded = decode(input, vk::link::max_frame_size);
    if (!decoded.empty()) {
        check(decode(encode(decoded), decoded.size()) == decoded);
    }
    // Into a buffer one byte short of the decoded size: refused, never overrun.
    if (decoded.size() > 1) {
        check(decode(input, decoded.size() - 1).empty());
    }

    if (!input.empty()) {
        check(decode(encode(input), input.size()) == input);
    }
    return 0;
}
