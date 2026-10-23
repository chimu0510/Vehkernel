// SPDX-License-Identifier: GPL-3.0-or-later
// What the Vehkernel bootloader needs from a platform (ADR-0023), after
// ArduPilot's Tools/AP_Bootloader/support.h.
//
// The bootloader core is a template over a type satisfying this concept, so
// the STM32 bootloader, the SITL bootloader and the unit tests' fake each
// provide their own (ADR-0009). Firmware offsets count from the start of the
// FIRMWARE region (ADR-0020); the platform keeps every write and erase
// inside it.
#pragma once

#include <concepts>
#include <cstdint>
#include <span>
#include <string_view>

namespace vk::bootloader {

template <class P>
concept Platform = requires(P& p, const P& cp, std::span<const std::uint8_t> bytes,
                            std::span<const std::uint32_t> words, std::uint32_t value) {
    // A byte from the ports the board lists, or -1 when none arrives within
    // `value` milliseconds (0: do not wait).
    { p.read_byte(value) } -> std::same_as<int>;
    // Sends on the port the last byte came from.
    p.write(bytes);
    // Changes the baud rate of that port.
    p.set_baud(value);
    // From now on, listens on that port only.
    p.lock_port();

    { p.millis() } -> std::same_as<std::uint32_t>;
    p.delay_ms(value);

    { cp.firmware_size() } -> std::same_as<std::uint32_t>;
    { cp.read_word(value) } -> std::same_as<std::uint32_t>;  // offset, a multiple of 4
    { p.erase_firmware() } -> std::same_as<bool>;
    { p.write_words(value, words) } -> std::same_as<bool>;  // offset, a multiple of 4

    // The request the firmware left before resetting (RTC backup register 0).
    { cp.boot_signal() } -> std::same_as<std::uint32_t>;
    p.set_boot_signal(value);
    // The board's stay-in-bootloader input is active.
    { cp.stay_requested() } -> std::same_as<bool>;

    { cp.mcu_id() } -> std::same_as<std::uint32_t>;  // DBGMCU IDCODE
    { cp.mcu_description() } -> std::convertible_to<std::string_view>;  // "family,revision"
    { cp.serial_word(value) } -> std::same_as<std::uint32_t>;  // byte offset into the unique ID
    { cp.otp_word(value) } -> std::same_as<std::uint32_t>;  // byte offset into OTP

    // Starts the firmware; returns only when the platform finds it cannot.
    p.start_firmware();
    // A line for whoever watches the bootloader: SITL prints it, a board drops it.
    p.report(std::string_view{});
};

}  // namespace vk::bootloader
