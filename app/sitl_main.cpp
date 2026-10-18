// SITL entry point: the firmware on the FreeRTOS POSIX port (ADR-0018).
//
// Usage: vehkernel-sitl [--duration <seconds>]
// The run stops by itself after the duration (default 1 s).
#include <vk/config.h>
#include <vk/core/scheduler.hpp>
#include <vk/platform/platform.hpp>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <span>
#include <string_view>

#ifndef CONFIG_PLATFORM_SITL
#error "the Kconfig layers must select the SITL platform for SITL builds"
#endif

namespace {

constexpr double default_duration_s = 1.0;

bool parse_duration(std::span<char*> args, double& duration_s)
{
    for (std::size_t i = 1; i < args.size(); ++i) {
        const std::string_view arg = args[i];
        if (arg == "--duration" && i + 1 < args.size()) {
            char* end = nullptr;
            duration_s = std::strtod(args[++i], &end);
            if (*end != '\0' || duration_s <= 0.0) {
                return false;
            }
        } else {
            return false;
        }
    }
    return true;
}

}  // namespace

int main(int argc, char** argv)
{
    double duration_s = default_duration_s;
    if (!parse_duration(std::span(argv, static_cast<std::size_t>(argc)), duration_s)) {
        std::fputs("usage: vehkernel-sitl [--duration <seconds>]\n", stderr);
        return 2;
    }
    vk::platform::init();
    vk::platform::stop_after(static_cast<std::uint64_t>(duration_s * 1e6));
    vk::core::start_scheduler();
    return 0;
}
