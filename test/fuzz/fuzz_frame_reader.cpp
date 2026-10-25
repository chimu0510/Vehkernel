// SPDX-License-Identifier: GPL-3.0-or-later
// Fuzz target for the frame reader (ADR-0002, ADR-0018): COBS, length, CRC,
// frame type and sequence checks on a byte stream from the peer. The first
// input byte sets the size of the chunks the rest is fed in, so a frame can
// straddle two reads as it does on a serial port.
//
// Checks, besides the sanitizers:
// - every frame passed on carries a known type and a body no larger than
//   the largest body;
// - every frame passed on, written again with the same type and body, is
//   read back the same;
// - each delimiter ends at most one frame, counted once in the statistics.
#include <vk/link/frame.hpp>

#include <algorithm>
#include <array>
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

}  // namespace

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size)
{
    if (size == 0) {
        return 0;
    }
    const std::vector<std::uint8_t> input(data + 1, data + size);
    const std::size_t chunk = std::max<std::size_t>(data[0], 1);

    vk::link::FrameReader reader;
    std::uint32_t passed = 0;
    for (std::size_t at = 0; at < input.size(); at += chunk) {
        const auto bytes = std::span(input).subspan(at, std::min(chunk, input.size() - at));
        reader.feed(bytes, [&](const vk::link::Frame& frame) {
            ++passed;
            check(frame.body.size() <= vk::link::max_body_size);
            check(vk::link::to_frame_type(static_cast<std::uint8_t>(frame.type)) == frame.type);

            vk::link::FrameWriter writer;
            std::array<std::uint8_t, vk::link::max_wire_size> wire{};
            const std::size_t wire_size = writer.write(frame.type, frame.body, wire);
            check(wire_size != 0);
            vk::link::FrameReader again;
            std::uint32_t read_back = 0;
            again.feed(std::span(wire).first(wire_size), [&](const vk::link::Frame& copy) {
                ++read_back;
                check(copy.type == frame.type);
                check(std::ranges::equal(copy.body, frame.body));
            });
            check(read_back == 1);
        });
    }

    const vk::link::ReaderStats& stats = reader.stats();
    const auto delimiters = static_cast<std::uint32_t>(std::ranges::count(input, std::uint8_t{0}));
    check(stats.frames == passed);
    check(stats.frames + stats.crc_errors + stats.malformed <= delimiters);
    return 0;
}
