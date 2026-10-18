#include <vk/platform/pty_transport.hpp>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace vk::platform {
namespace {

// Bytes a pty holds for the reading tool (the line discipline's buffer).
constexpr int pty_buffer_size = 4096;

}  // namespace

bool PtyTransport::open()
{
    controller_ = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (controller_ < 0 || ::grantpt(controller_) != 0 || ::unlockpt(controller_) != 0 ||
        ::ptsname_r(controller_, path_.data(), path_.size()) != 0) {
        std::fprintf(stderr, "link: cannot create a pty: %s\n", std::strerror(errno));
        return false;
    }
    device_ = ::open(path_.data(), O_RDWR | O_NOCTTY | O_CLOEXEC);
    termios mode{};
    if (device_ < 0 || ::tcgetattr(device_, &mode) != 0) {
        std::fprintf(stderr, "link: cannot open %s: %s\n", path_.data(), std::strerror(errno));
        return false;
    }
    // Raw: every byte passes unchanged, nothing is echoed.
    ::cfmakeraw(&mode);
    if (::tcsetattr(device_, TCSANOW, &mode) != 0) {
        std::fprintf(stderr, "link: cannot set %s to raw mode: %s\n", path_.data(), std::strerror(errno));
        return false;
    }
    return true;
}

std::size_t PtyTransport::write_space() const
{
    // Bytes written but not yet read by the tool wait in the device side's input queue.
    int queued = 0;
    if (device_ < 0 || ::ioctl(device_, FIONREAD, &queued) != 0) {
        return 0;
    }
    return queued < pty_buffer_size ? static_cast<std::size_t>(pty_buffer_size - queued) : 0;
}

std::size_t PtyTransport::write(std::span<const std::uint8_t> data)
{
    const ssize_t written = ::write(controller_, data.data(), data.size());
    return written < 0 ? 0 : static_cast<std::size_t>(written);
}

std::size_t PtyTransport::read(std::span<std::uint8_t> out)
{
    if (controller_ < 0) {
        return 0;
    }
    const ssize_t size = ::read(controller_, out.data(), out.size());
    return size <= 0 ? 0 : static_cast<std::size_t>(size);
}

}  // namespace vk::platform
