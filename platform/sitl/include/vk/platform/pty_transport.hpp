// Pseudo-terminal Link transport for SITL (ADR-0018).
//
// Creates a pty in raw mode; tools open its path as they would a board's
// serial port, so they are tested against a real serial API (termios, baud
// rate, framing) without hardware. The pty disappears when SITL exits.
#pragma once

#include <vk/hal/transport.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace vk::platform {

class PtyTransport final : public hal::Transport {
public:
    PtyTransport() = default;
    PtyTransport(const PtyTransport&) = delete;
    PtyTransport& operator=(const PtyTransport&) = delete;
    PtyTransport(PtyTransport&&) = delete;
    PtyTransport& operator=(PtyTransport&&) = delete;
    ~PtyTransport() = default;

    // Creates the pty; false, with the reason on stderr, on failure.
    bool open();

    // The path tools open, such as /dev/pts/3.
    [[nodiscard]] const char* path() const { return path_.data(); }

    [[nodiscard]] std::size_t write_space() const override;
    std::size_t write(std::span<const std::uint8_t> data) override;
    std::size_t read(std::span<std::uint8_t> out) override;

private:
    int controller_ = -1;
    int device_ = -1;  // held open, so the controller never reads end-of-file between tools
    std::array<char, 64> path_{};
};

}  // namespace vk::platform
