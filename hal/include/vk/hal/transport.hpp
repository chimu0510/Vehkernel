// Link transport contract (ADR-0002, ADR-0009).
//
// A transport carries a Link's bytes: a UART on a board, a UDP socket or a
// pty in SITL. Which transport a Link uses is chosen at runtime, so this is a
// virtual interface. No call blocks.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace vk::hal {

class Transport {
public:
    Transport() = default;
    Transport(const Transport&) = delete;
    Transport& operator=(const Transport&) = delete;
    Transport(Transport&&) = delete;
    Transport& operator=(Transport&&) = delete;

    // Bytes write() accepts now.
    [[nodiscard]] virtual std::size_t write_space() const = 0;

    // Queues what fits of `data`; returns the bytes taken.
    virtual std::size_t write(std::span<const std::uint8_t> data) = 0;

    // Copies up to `out.size()` bytes that have arrived; returns the bytes copied.
    virtual std::size_t read(std::span<std::uint8_t> out) = 0;

    // The bytes last read held a hello. A transport that can hear several
    // peers (UDP) sends to the one they came from from now on.
    virtual void select_peer() {}

protected:
    ~Transport() = default;
};

}  // namespace vk::hal
