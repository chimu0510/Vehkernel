// SITL entry point: the firmware on the FreeRTOS POSIX port (ADR-0018).
//
// Usage: vehkernel-sitl [--duration <seconds>]
// The run stops by itself after the duration (default 1 s) and prints a
// report: one line per topic instance and per work item, as space-separated
// key=value pairs, so scripts can parse it.
#include <vk/config.h>
#include <vk/core/bus.hpp>
#include <vk/core/scheduler.hpp>
#include <vk/core/static_instance.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/drivers/icm20948.hpp>
#include <vk/hal/i2c.hpp>
#include <vk/msg/registry.hpp>
#include <vk/platform/platform.hpp>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <span>
#include <string_view>

#ifndef CONFIG_PLATFORM_SITL
#error "the Kconfig layers must select the SITL platform for SITL builds"
#endif
#ifndef CONFIG_DRIVER_ICM20948
#error "SITL simulates an ICM20948 and needs its driver"
#endif

namespace {

constexpr double default_duration_s = 1.0;

vk::core::StaticInstance<vk::drivers::Icm20948> imu;

// The real driver, against the simulated IMU on the simulated I2C bus.
void start_imu()
{
    imu.emplace(vk::hal::I2cDevice(vk::platform::imu_bus(), vk::platform::imu_address),
                vk::drivers::Icm20948::Config{.instance = 0, .rate_hz = vk::platform::imu_rate_hz},
                vk::core::work_queue(vk::core::WorkQueueTier::Ctrl));
    imu->start();
}

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

void print_report(double duration_s)
{
    std::printf("report duration_s=%.3f\n", duration_s);
    for (const auto* topic : vk::msg::all_topics()) {
        const auto status = topic->status();
        std::printf("topic name=%.*s instance=%zu publications=%llu rate_hz=%.2f last_update_us=%llu\n",
                    static_cast<int>(status.name.size()), status.name.data(), status.instance,
                    static_cast<unsigned long long>(status.publications), static_cast<double>(status.rate_hz),
                    static_cast<unsigned long long>(status.last_update_us));
    }
    for (const auto* item : vk::core::work_items()) {
        const auto stats = item->stats();
        const auto queue = item->queue().name();
        std::printf("work_item name=%.*s queue=%.*s runs=%u overruns=%u run_time_last_us=%u "
                    "run_time_max_us=%u run_time_mean_us=%u jitter_max_us=%u\n",
                    static_cast<int>(item->name().size()), item->name().data(), static_cast<int>(queue.size()),
                    queue.data(), stats.runs, stats.overruns, stats.run_time_last_us, stats.run_time_max_us,
                    stats.run_time_mean_us, stats.jitter_max_us);
    }
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
    start_imu();
    vk::platform::stop_after(static_cast<std::uint64_t>(duration_s * 1e6));
    vk::core::start_scheduler();
    print_report(duration_s);
    return 0;
}
