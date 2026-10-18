// SITL entry point: the firmware on the FreeRTOS POSIX port (ADR-0018).
//
// Usage: vehkernel-sitl [--duration <seconds>] [--link udp] [--udp <address>:<port>]
//
// The run stops by itself after the duration (default 1 s; 0 runs until
// interrupted) and prints a report: one line per topic instance and per work
// item, as space-separated key=value pairs, so scripts can parse it. A Link
// streams topics to the development host, over UDP by default
// (127.0.0.1:15600, port 0 for any free port); its line, with the port bound,
// is printed when it starts.
#include <vk/config.h>
#include <vk/core/bus.hpp>
#include <vk/core/scheduler.hpp>
#include <vk/core/static_instance.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/drivers/icm20948.hpp>
#include <vk/hal/i2c.hpp>
#include <vk/msg/registry.hpp>
#include <vk/platform/platform.hpp>

#ifdef CONFIG_LINK_UDP
#include <vk/link/link.hpp>
#include <vk/link/topic_map.hpp>
#include <vk/platform/udp_transport.hpp>
#endif

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iterator>
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

struct Options {
    double duration_s = default_duration_s;  // 0 runs until interrupted
    std::string_view link = "udp";
    std::array<char, 64> udp_address{"127.0.0.1"};  // NUL-terminated
    std::uint16_t udp_port = 15600;
};

bool parse_address(const char* argument, Options& options)
{
    const std::string_view text = argument;
    const auto colon = text.rfind(':');
    if (colon == std::string_view::npos || colon == 0) {
        return false;
    }
    const std::string_view address = text.substr(0, colon);
    const char* port = std::next(argument, static_cast<std::ptrdiff_t>(colon + 1));
    if (address.size() >= options.udp_address.size() || *port == '\0') {
        return false;
    }
    char* end = nullptr;
    const unsigned long value = std::strtoul(port, &end, 10);
    if (*end != '\0' || value > 65535) {
        return false;
    }
    options.udp_address.fill('\0');
    address.copy(options.udp_address.data(), address.size());
    options.udp_port = static_cast<std::uint16_t>(value);
    return true;
}

bool parse_options(std::span<char*> args, Options& options)
{
    for (std::size_t i = 1; i < args.size(); ++i) {
        const std::string_view arg = args[i];
        if (i + 1 >= args.size()) {
            return false;
        }
        const char* value = args[++i];
        if (arg == "--duration") {
            char* end = nullptr;
            options.duration_s = std::strtod(value, &end);
            if (*end != '\0' || options.duration_s < 0.0) {
                return false;
            }
        } else if (arg == "--link") {
            if (std::string_view(value) != "udp") {
                return false;
            }
            options.link = value;
        } else if (arg == "--udp") {
            if (!parse_address(value, options)) {
                return false;
            }
        } else {
            return false;
        }
    }
    return true;
}

#ifdef CONFIG_LINK_UDP
vk::platform::UdpTransport udp;
vk::core::StaticInstance<vk::link::Link> host_link;

bool start_link(const Options& options)
{
    if (!udp.open(options.udp_address.data(), options.udp_port)) {
        return false;
    }
    host_link.emplace("link0", udp, vk::link::default_topic_map(), vk::link::Link::Config{},
                 vk::core::work_queue(vk::core::WorkQueueTier::Io));
    host_link->start();
    std::printf("link transport=udp address=%s:%u\n", options.udp_address.data(), udp.port());
    std::fflush(stdout);
    return true;
}
#endif

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
    Options options;
    if (!parse_options(std::span(argv, static_cast<std::size_t>(argc)), options)) {
        std::fputs("usage: vehkernel-sitl [--duration <seconds>] [--link udp] [--udp <address>:<port>]\n", stderr);
        return 2;
    }
    vk::platform::init();
    start_imu();
#ifdef CONFIG_LINK_UDP
    if (!start_link(options)) {
        return 1;
    }
#endif
    if (options.duration_s > 0.0) {
        vk::platform::stop_after(static_cast<std::uint64_t>(options.duration_s * 1e6));
    }
    vk::core::start_scheduler();
    print_report(options.duration_s);
    return 0;
}
