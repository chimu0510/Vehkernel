#include <vk/platform/udp_transport.hpp>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>

namespace vk::platform {
namespace {

// The largest UDP payload over IPv4.
constexpr std::size_t max_datagram = 65507;

}  // namespace

bool UdpTransport::open(const char* address, std::uint16_t port)
{
    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_port = htons(port);
    if (inet_pton(AF_INET, address, &local.sin_addr) != 1) {
        std::fprintf(stderr, "link: '%s' is not an IPv4 address\n", address);
        return false;
    }
    socket_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (socket_ < 0) {
        std::fprintf(stderr, "link: cannot create a UDP socket: %s\n", std::strerror(errno));
        return false;
    }
    if (::bind(socket_, reinterpret_cast<const sockaddr*>(&local), sizeof(local)) != 0) {
        std::fprintf(stderr, "link: cannot bind UDP %s:%u: %s\n", address, port, std::strerror(errno));
        ::close(socket_);
        socket_ = -1;
        return false;
    }
    socklen_t local_size = sizeof(local);
    getsockname(socket_, reinterpret_cast<sockaddr*>(&local), &local_size);
    port_ = ntohs(local.sin_port);
    return true;
}

std::size_t UdpTransport::write_space() const
{
    return have_peer_ ? max_datagram : 0;
}

std::size_t UdpTransport::write(std::span<const std::uint8_t> data)
{
    if (!have_peer_ || data.size() > max_datagram) {
        return 0;
    }
    const ssize_t sent = ::sendto(socket_, data.data(), data.size(), 0, reinterpret_cast<const sockaddr*>(&peer_),
                                  sizeof(peer_));
    // A full socket buffer drops the datagram, as a full UART would.
    return sent < 0 ? 0 : static_cast<std::size_t>(sent);
}

std::size_t UdpTransport::read(std::span<std::uint8_t> out)
{
    if (socket_ < 0) {
        return 0;
    }
    sockaddr_in sender{};
    socklen_t sender_size = sizeof(sender);
    const ssize_t size = ::recvfrom(socket_, out.data(), out.size(), 0, reinterpret_cast<sockaddr*>(&sender),
                                    &sender_size);
    if (size <= 0) {
        return 0;
    }
    last_sender_ = sender;
    return static_cast<std::size_t>(size);
}

void UdpTransport::select_peer()
{
    peer_ = last_sender_;
    have_peer_ = true;
}

}  // namespace vk::platform
