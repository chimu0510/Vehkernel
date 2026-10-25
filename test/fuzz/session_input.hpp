// SPDX-License-Identifier: GPL-3.0-or-later
// The input format of the session fuzz target, shared with the seed
// generator. The frames reach Session::handle() as the frame reader would
// pass them on, without COBS or CRC, so the fuzzer spends its time on the
// frame bodies.
//
// One byte of options, then records until the input ends:
//
//   u8 type | u8 time step | u8 body length | body
//
// The time step advances the clock by that many times 20 ms before the
// frame is handled, so a run can let the peer time out (3 s). A record cut
// short by the end of the input carries the bytes that are left.
#pragma once

#include <cstdint>

namespace vk::fuzz {

// Bits of the options byte: how the fake vehicle control answers and
// whether the sink drops frames.
namespace session_options {
inline constexpr std::uint8_t refuse_mode = 1U << 0U;
inline constexpr std::uint8_t refuse_arming = 1U << 1U;
inline constexpr std::uint8_t refuse_bringup = 1U << 2U;
inline constexpr std::uint8_t refuse_reboot = 1U << 3U;
inline constexpr std::uint8_t drop_frames = 1U << 4U;    // the sink drops every third frame
inline constexpr std::uint8_t no_control = 1U << 5U;     // the session has no vehicle control
inline constexpr std::uint8_t no_parameters = 1U << 6U;  // the session has no parameters
}  // namespace session_options

inline constexpr std::uint64_t session_time_step_us = 20'000;

}  // namespace vk::fuzz
