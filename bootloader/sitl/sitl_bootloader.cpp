// SPDX-License-Identifier: GPL-3.0-or-later
// SITL bootloader (ADR-0018, ADR-0023): the Vehkernel bootloader on the
// development host, so CI flashes it with ArduPilot's uploader.py.
//
// Usage: vehkernel-sitl-bootloader --flash <path> --board-id <id>
//                                  [--firmware-size <bytes>] [--boot-signal <value>] [--stay]
//
// The FIRMWARE region is the file given with --flash, created blank (0xFF)
// at --firmware-size bytes (default 448 KiB) when missing, and written
// through on every change, so a run cut off midway leaves what a board would
// hold. The port is a pseudo-terminal; its path is printed first as
// "bootloader pty=<path>". --boot-signal sets the value the firmware would
// have left in the RTC backup register, and --stay holds the stay input.
// Each decision is printed as "bootloader <line>". Starting the firmware
// prints "start size=<image size>" and exits with status 0.
#include <vk/bootloader/bootloader.hpp>
#include <vk/core/app_descriptor.hpp>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace {

class SitlPlatform {
public:
    SitlPlatform(std::string flash_path, std::uint32_t firmware_size, std::uint32_t boot_signal, bool stay)
        : flash_path_(std::move(flash_path)), words_(firmware_size / 4, 0xFFFFFFFF), boot_signal_(boot_signal),
          stay_(stay)
    {
        std::ifstream in(flash_path_, std::ios::binary);
        if (in) {
            in.read(reinterpret_cast<char*>(words_.data()),  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
                    static_cast<std::streamsize>(words_.size() * 4));
        }
        save();
        start_ = std::chrono::steady_clock::now();
    }

    bool open_pty()
    {
        fd_ = posix_openpt(O_RDWR | O_NOCTTY);
        if (fd_ < 0 || grantpt(fd_) != 0 || unlockpt(fd_) != 0) {
            return false;
        }
        termios mode{};
        tcgetattr(fd_, &mode);
        cfmakeraw(&mode);
        tcsetattr(fd_, TCSANOW, &mode);
        std::printf("bootloader pty=%s\n", ptsname(fd_));
        std::fflush(stdout);
        return true;
    }

    int read_byte(std::uint32_t timeout_ms)
    {
        pollfd fds{.fd = fd_, .events = POLLIN, .revents = 0};
        // With no peer, a pty reports a hang-up at once; wait as a UART would.
        if (poll(&fds, 1, static_cast<int>(timeout_ms)) <= 0 || (fds.revents & POLLIN) == 0) {
            if ((fds.revents & POLLHUP) != 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(timeout_ms == 0 ? 1 : timeout_ms));
            }
            return -1;
        }
        std::uint8_t byte = 0;
        return ::read(fd_, &byte, 1) == 1 ? byte : -1;
    }

    void write(std::span<const std::uint8_t> bytes)
    {
        std::size_t done = 0;
        while (done < bytes.size()) {
            const ssize_t n = ::write(fd_, bytes.data() + done, bytes.size() - done);
            if (n <= 0) {
                return;
            }
            done += static_cast<std::size_t>(n);
        }
    }

    void set_baud(std::uint32_t /*baud*/) {}  // a pty has no baud rate
    void lock_port() {}

    [[nodiscard]] std::uint32_t millis() const
    {
        return static_cast<std::uint32_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start_).count());
    }

    void delay_ms(std::uint32_t ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

    [[nodiscard]] std::uint32_t firmware_size() const { return static_cast<std::uint32_t>(words_.size() * 4); }
    [[nodiscard]] std::uint32_t read_word(std::uint32_t offset) const { return words_.at(offset / 4); }

    bool erase_firmware()
    {
        std::ranges::fill(words_, 0xFFFFFFFF);
        save();
        return true;
    }

    // As NOR Flash, a write only clears bits.
    bool write_words(std::uint32_t offset, std::span<const std::uint32_t> words)
    {
        if (offset % 4 != 0 || offset / 4 + words.size() > words_.size()) {
            return false;
        }
        for (std::size_t i = 0; i < words.size(); ++i) {
            words_.at(offset / 4 + i) &= words[i];
        }
        save();
        return true;
    }

    [[nodiscard]] std::uint32_t boot_signal() const { return boot_signal_; }
    void set_boot_signal(std::uint32_t value) { boot_signal_ = value; }
    [[nodiscard]] bool stay_requested() const { return stay_; }

    [[nodiscard]] std::uint32_t mcu_id() const { return 0x10076413; }  // an STM32F40x, revision Z
    [[nodiscard]] std::string_view mcu_description() const { return "SITL,Z"; }
    [[nodiscard]] std::uint32_t serial_word(std::uint32_t offset) const { return 0x56530000U + offset; }
    [[nodiscard]] std::uint32_t otp_word(std::uint32_t /*offset*/) const { return 0xFFFFFFFF; }

    void start_firmware()
    {
        const auto size = read_word(image_size_offset());
        std::printf("start size=%u\n", static_cast<unsigned>(size));
        std::fflush(stdout);
        std::exit(0);
    }

    void report(std::string_view line)
    {
        std::printf("bootloader %.*s\n", static_cast<int>(line.size()), line.data());
        std::fflush(stdout);
    }

private:
    // The image size from the app descriptor, which start_firmware() follows a check of.
    [[nodiscard]] std::uint32_t image_size_offset() const
    {
        for (std::uint32_t offset = 0; offset + 8 <= firmware_size(); offset += 4) {
            if (std::memcmp(&words_.at(offset / 4), vk::core::app_descriptor_signature.data(), 8) == 0) {
                return offset + 16;
            }
        }
        return 0;
    }

    void save() const
    {
        std::ofstream out(flash_path_, std::ios::binary | std::ios::trunc);
        out.write(reinterpret_cast<const char*>(words_.data()),  // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
                  static_cast<std::streamsize>(words_.size() * 4));
    }

    std::string flash_path_;
    std::vector<std::uint32_t> words_;
    std::uint32_t boot_signal_;
    bool stay_;
    int fd_ = -1;
    std::chrono::steady_clock::time_point start_;
};

static_assert(vk::bootloader::Platform<SitlPlatform>);

int usage()
{
    std::fprintf(stderr, "usage: vehkernel-sitl-bootloader --flash <path> --board-id <id> "
                         "[--firmware-size <bytes>] [--boot-signal <value>] [--stay]\n");
    return 2;
}

}  // namespace

int main(int argc, char** argv)
{
    std::string flash;
    long board_id = -1;
    unsigned long firmware_size = 448UL * 1024;
    unsigned long boot_signal = 0;
    bool stay = false;
    const std::vector<std::string_view> args(argv + 1, argv + argc);
    for (std::size_t i = 0; i < args.size(); ++i) {
        const bool has_value = i + 1 < args.size();
        if (args[i] == "--stay") {
            stay = true;
        } else if (args[i] == "--flash" && has_value) {
            flash = args[++i];
        } else if (args[i] == "--board-id" && has_value) {
            board_id = std::strtol(std::string(args[++i]).c_str(), nullptr, 0);
        } else if (args[i] == "--firmware-size" && has_value) {
            firmware_size = std::strtoul(std::string(args[++i]).c_str(), nullptr, 0);
        } else if (args[i] == "--boot-signal" && has_value) {
            boot_signal = std::strtoul(std::string(args[++i]).c_str(), nullptr, 0);
        } else {
            return usage();
        }
    }
    if (flash.empty() || board_id < 1 || board_id > 65535 || firmware_size == 0 || firmware_size % 4 != 0) {
        return usage();
    }
    SitlPlatform platform(flash, static_cast<std::uint32_t>(firmware_size), static_cast<std::uint32_t>(boot_signal),
                          stay);
    if (!platform.open_pty()) {
        std::perror("vehkernel-sitl-bootloader: pty");
        return 1;
    }
    vk::bootloader::Bootloader<SitlPlatform> bootloader(
        platform, {.board_id = static_cast<std::uint16_t>(board_id), .board_revision = 0});
    bootloader.run();
}
