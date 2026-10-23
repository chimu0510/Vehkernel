// SPDX-License-Identifier: GPL-3.0-or-later
// UDP Link transport for SITL (ADR-0018).
//
// One frame per datagram on a non-blocking socket bound to a local address.
// The peer is whoever sent the last hello, so tools connect without any
// configuration on the SITL side.
#pragma once

#include <vk/hal/transport.hpp>

#include <netinet/in.h>

#include <cstddef>
#include <cstdint>
#include <span>

namespace vk::platform {

class UdpTransport final : public hal::Transport {
public:
    static constexpr std::uint16_t default_port = 15600;

    UdpTransport() = default;
    UdpTransport(const UdpTransport&) = delete;
    UdpTransport& operator=(const UdpTransport&) = delete;
    UdpTransport(UdpTransport&&) = delete;
    UdpTransport& operator=(UdpTransport&&) = delete;
    ~UdpTransport() = default;

    // Binds to an IPv4 address and port, any free port if 0; false, with the
    // reason on stderr, on failure.
    bool open(const char* address, std::uint16_t port);

    // The port bound to.
    [[nodiscard]] std::uint16_t port() const { return port_; }

    [[nodiscard]] std::size_t write_space() const override;
    std::size_t write(std::span<const std::uint8_t> data) override;
    std::size_t read(std::span<std::uint8_t> out) override;
    void select_peer() override;

private:
    int socket_ = -1;
    std::uint16_t port_ = 0;
    sockaddr_in last_sender_{};
    sockaddr_in peer_{};
    bool have_peer_ = false;
};

}  // namespace vk::platform
