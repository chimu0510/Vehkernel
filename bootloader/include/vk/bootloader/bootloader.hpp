// SPDX-License-Identifier: GPL-3.0-or-later
// The Vehkernel bootloader (ADR-0023): a port of the structure of
// ArduPilot's Tools/AP_Bootloader at the commit in bootloader/UPSTREAM,
// written for any Platform (platform.hpp).
//
// - The protocol is ArduPilot's and PX4's at revision 5 (bl_protocol.cpp):
//   GET_SYNC, GET_DEVICE, CHIP_ERASE, PROG_MULTI, READ_MULTI, GET_CRC,
//   GET_OTP, GET_SN, GET_CHIP, GET_CHIP_DES, SET_BAUD and BOOT. Erasing and
//   programming need GET_SYNC and GET_DEVICE first, so noise on a UART
//   cannot erase the firmware. The first 32 bytes of the image are held
//   back and written by BOOT, so an interrupted upload leaves no runnable
//   image.
// - The boot decision is ArduPilot's (AP_Bootloader.cpp main): stay with
//   RTC_BOOT_HOLD or the stay input; start at once with RTC_BOOT_FAST;
//   otherwise listen for boot_window_ms and start a valid firmware, and
//   stay with an invalid one. The request is cleared once read.
// - A firmware starts only when check_firmware() passes and its first words
//   are written (jump_to_app).
#pragma once

#include <vk/bootloader/firmware_check.hpp>
#include <vk/bootloader/platform.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

namespace vk::bootloader {

inline constexpr std::uint32_t protocol_revision = 5;  // BL_PROTOCOL_VERSION

// Values the firmware leaves in RTC backup register 0 (ArduPilot's rtc_boot_magic).
inline constexpr std::uint32_t boot_signal_off = 0;
inline constexpr std::uint32_t boot_signal_hold = 0xb0070001;  // RTC_BOOT_HOLD
inline constexpr std::uint32_t boot_signal_fast = 0xb0070002;  // RTC_BOOT_FAST

// How long the bootloader listens at boot before starting a valid firmware.
inline constexpr std::uint32_t boot_window_ms = 1000;

namespace proto {
inline constexpr std::uint8_t insync = 0x12;
inline constexpr std::uint8_t eoc = 0x20;
inline constexpr std::uint8_t ok = 0x10;
inline constexpr std::uint8_t failed = 0x11;
inline constexpr std::uint8_t invalid = 0x13;

inline constexpr std::uint8_t get_sync = 0x21;
inline constexpr std::uint8_t get_device = 0x22;
inline constexpr std::uint8_t chip_erase = 0x23;
inline constexpr std::uint8_t prog_multi = 0x27;
inline constexpr std::uint8_t read_multi = 0x28;
inline constexpr std::uint8_t get_crc = 0x29;
inline constexpr std::uint8_t get_otp = 0x2a;
inline constexpr std::uint8_t get_sn = 0x2b;
inline constexpr std::uint8_t get_chip = 0x2c;
inline constexpr std::uint8_t get_chip_des = 0x2e;
inline constexpr std::uint8_t boot = 0x30;
inline constexpr std::uint8_t debug = 0x31;
inline constexpr std::uint8_t set_baud = 0x33;
inline constexpr std::uint8_t extf_erase = 0x34;
inline constexpr std::uint8_t extf_prog_multi = 0x35;
inline constexpr std::uint8_t extf_read_multi = 0x36;
inline constexpr std::uint8_t extf_get_crc = 0x37;
inline constexpr std::uint8_t chip_full_erase = 0x40;

// GET_DEVICE arguments.
inline constexpr std::uint8_t device_bl_rev = 1;
inline constexpr std::uint8_t device_board_id = 2;
inline constexpr std::uint8_t device_board_rev = 3;
inline constexpr std::uint8_t device_fw_size = 4;
inline constexpr std::uint8_t device_vec_area = 5;
inline constexpr std::uint8_t device_extf_size = 6;

inline constexpr std::size_t prog_multi_max = 256;  // the flash buffer
inline constexpr std::size_t reserve_lead_words = 8;
}  // namespace proto

template <Platform P>
class Bootloader {
public:
    struct Config {
        std::uint16_t board_id = 0;
        std::uint32_t board_revision = 0;
    };

    // What to do at boot: start the firmware at once, then serve the
    // protocol for window_ms (0: until flashed or reset).
    struct Decision {
        bool start_at_once = false;
        std::uint32_t window_ms = 0;
        friend bool operator==(const Decision&, const Decision&) = default;
    };

    enum class Step : std::uint8_t {
        Continue,  // a command was handled, or nothing arrived yet
        TimedOut,  // the window ran out with no valid conversation
        Boot,      // BOOT: the upload is complete
    };

    Bootloader(P& platform, const Config& config) : p_(platform), config_(config) {}

    // Never returns on a board: the firmware starts, or the bootloader serves.
    void run()
    {
        const Decision decision = boot_decision();
        if (decision.start_at_once) {
            try_start();
        }
        while (true) {
            serve(decision.window_ms);
            try_start();
        }
    }

    // Reads and clears the firmware's request, checks the firmware and decides.
    Decision boot_decision()
    {
        const std::uint32_t signal = p_.boot_signal();
        p_.set_boot_signal(boot_signal_off);
        Decision decision{.start_at_once = false, .window_ms = boot_window_ms};
        if (signal == boot_signal_hold) {
            decision.window_ms = 0;
        } else if (signal == boot_signal_fast) {
            decision = {.start_at_once = true, .window_ms = 0};
        }
        const FirmwareCheck check = check_firmware(p_, config_.board_id);
        if (check != FirmwareCheck::Ok || p_.stay_requested()) {
            decision = {.start_at_once = false, .window_ms = 0};
        }
        report_decision(check, decision.start_at_once, decision.window_ms);
        return decision;
    }

    // Serves the protocol until `timeout_ms` passes with no valid
    // conversation (0: forever) or the host sends BOOT.
    void serve(std::uint32_t timeout_ms)
    {
        begin(timeout_ms);
        while (step() == Step::Continue) {
        }
    }

    // Starts a serving session; step() then handles one command at a time.
    void begin(std::uint32_t timeout_ms)
    {
        address_ = p_.firmware_size();  // force an erase before programming works
        read_address_ = 0;
        first_words_.fill(0xFFFFFFFF);
        done_sync_ = false;
        device_flags_ = 0;
        done_erase_ = false;
        original_timeout_ = timeout_ms;
        timeout_ = timeout_ms;
        start_ms_ = p_.millis();
    }

    // Waits without blocking for a command byte and handles the command.
    Step step()
    {
        if (timeout_ != 0 && p_.millis() - start_ms_ >= timeout_) {
            return Step::TimedOut;
        }
        const int c = p_.read_byte(0);
        if (c < 0) {
            return Step::Continue;
        }
        return handle(static_cast<std::uint8_t>(c));
    }

    // A firmware that passes the check starts; otherwise this returns.
    void try_start()
    {
        const FirmwareCheck check = check_firmware(p_, config_.board_id);
        if (check != FirmwareCheck::Ok) {
            report_check("start refused", check);
            return;
        }
        // The lead words are written last, by BOOT: an image without them never ran to completion.
        for (std::uint32_t i = 0; i < proto::reserve_lead_words; ++i) {
            if (p_.read_word(i * 4) == 0xFFFFFFFF) {
                p_.report("start refused: lead words not written");
                return;
            }
        }
        p_.start_firmware();
    }

private:
    enum class Outcome : std::uint8_t { Good, Bad, Fail, Silent, Boot, BaudChanged };

    Step handle(std::uint8_t c)
    {
        const Outcome outcome = command(c);
        switch (outcome) {
        case Outcome::Good:
            p_.lock_port();
            if (device_finished()) {
                timeout_ = 0;  // a host is talking: stay until it is done
            }
            respond(proto::ok);
            break;
        case Outcome::Bad:
            // Could be noise on a UART: keep the original window unless an erase began.
            if (!done_erase_) {
                timeout_ = original_timeout_;
            }
            respond(proto::invalid);
            break;
        case Outcome::Fail:
            respond(proto::failed);
            break;
        case Outcome::Boot:
            return Step::Boot;
        case Outcome::Silent:
        case Outcome::BaudChanged:
            break;
        }
        return Step::Continue;
    }

    Outcome command(std::uint8_t c)
    {
        switch (c) {
        case proto::get_sync:
            if (!eoc(2)) {
                return Outcome::Bad;
            }
            done_sync_ = true;
            return Outcome::Good;
        case proto::get_device:
            return get_device();
        case proto::chip_erase:
        case proto::chip_full_erase:
            return erase();
        case proto::prog_multi:
            return program();
        case proto::get_crc:
            return get_crc();
        case proto::read_multi:
            return read_multi();
        case proto::get_otp:
        case proto::get_sn:
            return read_id_word(c);
        case proto::get_chip:
            if (!eoc(2)) {
                return Outcome::Bad;
            }
            send_word(p_.mcu_id());
            return Outcome::Good;
        case proto::get_chip_des:
            return get_chip_description();
        case proto::boot:
            return boot();
        case proto::debug:
            return Outcome::Good;
        case proto::set_baud:
            return set_baud();
        case proto::extf_erase:
        case proto::extf_prog_multi:
        case proto::extf_read_multi:
        case proto::extf_get_crc:
            return Outcome::Bad;  // no external Flash
        default:
            return Outcome::Silent;  // not a command: ignore, as ArduPilot does
        }
    }

    Outcome get_device()
    {
        const int arg = p_.read_byte(1000);
        if (arg < 0 || !eoc(2)) {
            return Outcome::Bad;
        }
        read_address_ = 0;
        switch (arg) {
        case proto::device_bl_rev:
            send_word(protocol_revision);
            break;
        case proto::device_board_id:
            send_word(config_.board_id);
            break;
        case proto::device_board_rev:
            send_word(config_.board_revision);
            break;
        case proto::device_fw_size:
            send_word(p_.firmware_size());
            break;
        case proto::device_vec_area:
            for (std::uint32_t vector = 7; vector <= 10; ++vector) {
                send_word(p_.read_word(vector * 4));
            }
            break;
        case proto::device_extf_size:
            send_word(0);
            break;
        default:
            return Outcome::Bad;
        }
        device_flags_ |= static_cast<std::uint8_t>(1U << static_cast<unsigned>(arg - 1));
        return Outcome::Good;
    }

    [[nodiscard]] bool device_finished() const
    {
        // GET_SYNC, and GET_DEVICE for the bootloader revision, board ID and firmware size.
        return done_sync_ && (device_flags_ & 0x0BU) == 0x0BU;
    }

    Outcome erase()
    {
        if (!device_finished() || !eoc(2)) {
            return Outcome::Bad;
        }
        done_erase_ = true;
        timeout_ = 0;  // no going back
        if (!p_.erase_firmware()) {
            return Outcome::Fail;
        }
        for (std::uint32_t offset = 0; offset < p_.firmware_size(); offset += 4) {
            if (p_.read_word(offset) != 0xFFFFFFFF) {
                return Outcome::Fail;
            }
        }
        address_ = 0;
        return Outcome::Good;
    }

    Outcome program()
    {
        if (!device_finished()) {
            return Outcome::Bad;
        }
        const int count = p_.read_byte(50);
        if (count < 0 || count % 4 != 0 || address_ + static_cast<std::uint32_t>(count) > p_.firmware_size() ||
            static_cast<std::size_t>(count) > buffer_.size() * 4) {
            return Outcome::Bad;
        }
        const auto words = static_cast<std::size_t>(count) / 4;
        for (std::size_t w = 0; w < words; ++w) {
            std::uint32_t word = 0;
            for (unsigned b = 0; b < 4; ++b) {
                const int byte = p_.read_byte(1000);
                if (byte < 0) {
                    return Outcome::Bad;
                }
                word |= static_cast<std::uint32_t>(byte) << (8U * b);
            }
            buffer_.at(w) = word;
        }
        if (!eoc(200)) {
            return Outcome::Bad;
        }
        // Hold back the lead words until BOOT.
        for (std::size_t w = 0; w < words; ++w) {
            const std::uint32_t index = address_ / 4 + static_cast<std::uint32_t>(w);
            if (index < proto::reserve_lead_words) {
                first_words_.at(index) = buffer_.at(w);
                buffer_.at(w) = 0xFFFFFFFF;
            }
        }
        if (!p_.write_words(address_, std::span(buffer_).first(words))) {
            return Outcome::Fail;
        }
        address_ += static_cast<std::uint32_t>(count);
        return Outcome::Good;
    }

    Outcome get_crc()
    {
        if (!eoc(2)) {
            return Outcome::Bad;
        }
        std::uint32_t sum = 0;
        for (std::uint32_t offset = 0; offset < p_.firmware_size(); offset += 4) {
            const std::uint32_t index = offset / 4;
            const bool held = index < proto::reserve_lead_words && first_words_[0] != 0xFFFFFFFF;
            sum = crc32_word(sum, held ? first_words_.at(index) : p_.read_word(offset));
        }
        send_word(sum);
        return Outcome::Good;
    }

    Outcome read_multi()
    {
        const int count = p_.read_byte(50);
        if (count < 0 || count % 4 != 0 || read_address_ + static_cast<std::uint32_t>(count) > p_.firmware_size() ||
            !eoc(2)) {
            return Outcome::Bad;
        }
        for (int i = 0; i < count / 4; ++i) {
            send_word(p_.read_word(read_address_));
            read_address_ += 4;
        }
        return Outcome::Good;
    }

    Outcome read_id_word(std::uint8_t c)
    {
        const std::optional<std::uint32_t> index = read_word_arg(100);
        if (!index || !eoc(2)) {
            return Outcome::Bad;
        }
        send_word(c == proto::get_otp ? p_.otp_word(*index) : p_.serial_word(*index));
        return Outcome::Good;
    }

    Outcome get_chip_description()
    {
        if (!eoc(2)) {
            return Outcome::Bad;
        }
        const std::string_view description = p_.mcu_description();
        send_word(static_cast<std::uint32_t>(description.size()));
        for (const char ch : description) {
            const std::array<std::uint8_t, 1> byte{static_cast<std::uint8_t>(ch)};
            p_.write(byte);
        }
        return Outcome::Good;
    }

    Outcome boot()
    {
        if (!eoc(1000)) {
            return Outcome::Bad;
        }
        if (first_words_[0] != 0xFFFFFFFF) {
            if (!p_.write_words(0, first_words_)) {
                return Outcome::Fail;
            }
            first_words_.fill(0xFFFFFFFF);
        }
        respond(proto::ok);
        p_.delay_ms(100);  // let the reply leave
        return Outcome::Boot;
    }

    Outcome set_baud()
    {
        if (!device_finished()) {
            return Outcome::Bad;
        }
        const std::optional<std::uint32_t> baud = read_word_arg(100);
        if (!baud || !eoc(2)) {
            return Outcome::Bad;
        }
        respond(proto::ok);
        p_.delay_ms(5);
        p_.set_baud(*baud);
        p_.lock_port();
        timeout_ = 0;
        return Outcome::BaudChanged;
    }

    bool eoc(std::uint32_t timeout_ms) { return p_.read_byte(timeout_ms) == proto::eoc; }

    std::optional<std::uint32_t> read_word_arg(std::uint32_t timeout_ms)
    {
        std::uint32_t word = 0;
        for (unsigned b = 0; b < 4; ++b) {
            const int byte = p_.read_byte(timeout_ms);
            if (byte < 0) {
                return std::nullopt;
            }
            word |= static_cast<std::uint32_t>(byte) << (8U * b);
        }
        return word;
    }

    void send_word(std::uint32_t word)
    {
        const std::array<std::uint8_t, 4> bytes{static_cast<std::uint8_t>(word), static_cast<std::uint8_t>(word >> 8U),
                                                static_cast<std::uint8_t>(word >> 16U),
                                                static_cast<std::uint8_t>(word >> 24U)};
        p_.write(bytes);
    }

    void respond(std::uint8_t status)
    {
        const std::array<std::uint8_t, 2> reply{proto::insync, status};
        p_.write(reply);
    }

    void report_check(std::string_view what, FirmwareCheck check)
    {
        std::array<char, 64> line{};
        std::size_t n = 0;
        for (const std::string_view part : {what, std::string_view(": "), to_string(check)}) {
            for (const char ch : part) {
                if (n < line.size()) {
                    line.at(n++) = ch;
                }
            }
        }
        p_.report(std::string_view(line.data(), n));
    }

    void report_decision(FirmwareCheck check, bool try_boot, std::uint32_t timeout)
    {
        if (try_boot) {
            report_check("start at once", check);
        } else if (timeout == 0) {
            report_check("stay", check);
        } else {
            report_check("listen", check);
        }
    }

    P& p_;
    Config config_;
    std::array<std::uint32_t, proto::prog_multi_max / 4> buffer_{};
    std::array<std::uint32_t, proto::reserve_lead_words> first_words_{};
    std::uint32_t address_ = 0;
    std::uint32_t read_address_ = 0;
    bool done_sync_ = false;
    std::uint8_t device_flags_ = 0;
    bool done_erase_ = false;
    std::uint32_t original_timeout_ = 0;
    std::uint32_t timeout_ = 0;
    std::uint32_t start_ms_ = 0;
};

}  // namespace vk::bootloader
