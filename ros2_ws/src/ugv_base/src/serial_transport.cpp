#include "ugv_base/serial_transport.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <poll.h>
#include <cstring>
#include <cstdio>

namespace ugv::base {

namespace {
speed_t get_termios_baud(int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default: return B115200;
    }
}
} // namespace

SerialTransport::SerialTransport() = default;

SerialTransport::~SerialTransport() {
    close();
}

bool SerialTransport::open(const std::string& device, int baud_rate) {
    close();

    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        return false;
    }

    struct termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        close();
        return false;
    }

    speed_t speed = get_termios_baud(baud_rate);
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    // 8N1 raw mode
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_cflag &= ~PARENB;                      // no parity
    tty.c_cflag &= ~CSTOPB;                      // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                     // no hardware flow control
    tty.c_cflag |= (CLOCAL | CREAD);             // ignore modem controls, enable read

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // raw input
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // no software flow control
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;                          // raw output

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        close();
        return false;
    }

    // Flush any residual bytes
    tcflush(fd_, TCIOFLUSH);
    return true;
}

void SerialTransport::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SerialTransport::is_open() const {
    return fd_ >= 0;
}

ssize_t SerialTransport::write(const uint8_t* data, size_t size) {
    if (!is_open()) {
        return -1;
    }
    return ::write(fd_, data, size);
}

ssize_t SerialTransport::read(uint8_t* buffer, size_t max_size, int timeout_ms) {
    if (!is_open()) {
        return -1;
    }

    struct pollfd pfd{};
    pfd.fd = fd_;
    pfd.events = POLLIN;

    int ret = poll(&pfd, 1, timeout_ms);
    if (ret > 0 && (pfd.revents & POLLIN)) {
        return ::read(fd_, buffer, max_size);
    }
    return (ret == 0) ? 0 : -1;
}

} // namespace ugv::base
