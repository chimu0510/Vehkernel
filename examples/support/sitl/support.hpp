// SPDX-License-Identifier: GPL-3.0-or-later
// Support code for SITL board examples (spec 0013, ADR-0025).
#pragma once

#include <vk/core/static_instance.hpp>
#include <vk/drivers/ssd1306.hpp>
#include <vk/hal/clock.hpp>
#include <vk/platform/platform.hpp>
#include <vk/sim/ssd1306.hpp>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <string_view>
#include <type_traits>

namespace example {

namespace detail {

inline vk::core::StaticInstance<vk::drivers::Ssd1306> display_driver;
inline std::uint64_t stop_duration_us = 0;
inline bool has_stopped = false;

}  // namespace detail

// Emits a key=value report line to standard output in SITL, in the form sitl_main.cpp uses.
template <typename T>
inline void report(std::string_view key, const T& value)
{
    if constexpr (std::is_integral_v<T>) {
        if constexpr (std::is_signed_v<T>) {
            std::printf("%.*s=%lld\n", static_cast<int>(key.size()), key.data(), static_cast<long long>(value));
        } else {
            std::printf("%.*s=%llu\n", static_cast<int>(key.size()), key.data(), static_cast<unsigned long long>(value));
        }
    } else if constexpr (std::is_floating_point_v<T>) {
        std::printf("%.*s=%.3f\n", static_cast<int>(key.size()), key.data(), static_cast<double>(value));
    } else {
        const std::string_view s(value);
        std::printf("%.*s=%.*s\n", static_cast<int>(key.size()), key.data(), static_cast<int>(s.size()), s.data());
    }
    std::fflush(stdout);
}

// Initializes the SITL platform and parses command-line options.
inline void init(int argc = 0, char** argv = nullptr)
{
    vk::platform::init();
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg(argv[i]);
        if (arg == "--duration" && i + 1 < argc) {
            const double duration_s = std::strtod(argv[++i], nullptr);
            if (duration_s > 0.0) {
                detail::stop_duration_us = static_cast<std::uint64_t>(duration_s * 1e6);
            }
        } else if (arg == "--display" && i + 1 < argc) {
            vk::platform::display().write_image_to(argv[++i]);
        }
    }
}

// Returns whether the example loop should continue running. When the duration
// configured with --duration expires, reports display_on and returns false.
inline bool running()
{
    if (detail::stop_duration_us > 0 && vk::hal::SystemClock::now_us() >= detail::stop_duration_us) {
        if (!detail::has_stopped) {
            detail::has_stopped = true;
            report("display_on", vk::platform::display().displaying() ? 1 : 0);
        }
        return false;
    }
    return true;
}

// Sleeps until the system microsecond clock reaches target_us without consuming CPU.
inline void sleep_until_us(std::uint64_t target_us)
{
    const std::uint64_t now_us = vk::hal::SystemClock::now_us();
    if (now_us < target_us) {
        const std::uint64_t diff_us = target_us - now_us;
        struct timespec ts{};
        ts.tv_sec = static_cast<time_t>(diff_us / 1'000'000ULL);
        ts.tv_nsec = static_cast<long>((diff_us % 1'000'000ULL) * 1'000L);
        nanosleep(&ts, nullptr);
    }
}

// Returns the simulated SSD1306 OLED display driver.
inline vk::drivers::Ssd1306& display()
{
    if (!detail::display_driver.has_value()) {
        detail::display_driver.emplace(vk::hal::DisplayDevice(vk::platform::display()));
    }
    return *detail::display_driver;
}

}  // namespace example
