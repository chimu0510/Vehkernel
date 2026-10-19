// SITL entry point: the firmware on the FreeRTOS POSIX port (ADR-0018).
//
// Usage: vehkernel-sitl [--duration <seconds>] [--link udp|pty] [--udp <address>:<port>]
//                       [--motion still|tilt|rotate|shake] [--param-file <path>]
//
// The run stops by itself after the duration (default 1 s; 0 runs until
// interrupted) and prints a report: one line per topic instance and per work
// item, as space-separated key=value pairs, so scripts can parse it. A Link
// streams topics to the development host, over UDP by default
// (127.0.0.1:15600, port 0 for any free port) or over a pty with --link pty.
// Its line, with the port bound or the pty's path, is printed when it starts.
// The simulated IMU follows a scripted motion (level and still by default),
// and its true attitude is published on vehicle_attitude_groundtruth.
// Parameters are stored in a simulated NOR Flash: blank at every start, so
// runs are reproducible, or kept in the file given with --param-file.
#include <vk/config.h>
#include <vk/core/bus.hpp>
#include <vk/core/scheduler.hpp>
#include <vk/core/static_instance.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/drivers/icm20948.hpp>
#ifdef CONFIG_ESTIMATOR_MAHONY
#include <vk/estimation/attitude_estimator.hpp>
#endif
#include <vk/hal/i2c.hpp>
#include <vk/msg/registry.hpp>
#include <vk/params/params.hpp>
#ifdef CONFIG_PARAM_STORAGE_FLASH
#include <vk/core/parameter_storage.hpp>
#include <vk/hal/flash.hpp>
#include <vk/sim/nor_flash.hpp>
#endif
#include <vk/platform/platform.hpp>

#ifdef CONFIG_LINK
#include <vk/hal/transport.hpp>
#include <vk/link/link.hpp>
#include <vk/link/topic_map.hpp>
#endif
#ifdef CONFIG_LINK_UDP
#include <vk/platform/udp_transport.hpp>
#endif
#ifdef CONFIG_LINK_PTY
#include <vk/platform/pty_transport.hpp>
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
                vk::drivers::Icm20948::Config{
                    .instance = 0,
                    .board_orientation = static_cast<vk::hal::Rotation>(vk::params::sens.board_rot),
                    .rate_hz = vk::platform::imu_rate_hz},
                vk::core::work_queue(vk::core::WorkQueueTier::Ctrl));
    imu->start();
}

struct Options {
    double duration_s = default_duration_s;  // 0 runs until interrupted
    std::string_view link = "udp";
    std::array<char, 64> udp_address{"127.0.0.1"};  // NUL-terminated
    std::uint16_t udp_port = 15600;
    vk::sim::MotionProfile motion = vk::sim::MotionProfile::Still;
    const char* param_file = nullptr;  // nullptr: blank Flash in memory
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
            if (std::string_view(value) != "udp" && std::string_view(value) != "pty") {
                return false;
            }
            options.link = value;
        } else if (arg == "--motion") {
            const auto profile = vk::sim::motion_profile(value);
            if (!profile) {
                return false;
            }
            options.motion = *profile;
        } else if (arg == "--udp") {
            if (!parse_address(value, options)) {
                return false;
            }
        } else if (arg == "--param-file") {
            options.param_file = value;
        } else {
            return false;
        }
    }
    return true;
}

#ifdef CONFIG_LINK
vk::core::StaticInstance<vk::link::Link> host_link;
#endif
#ifdef CONFIG_LINK_UDP
vk::platform::UdpTransport udp;
#endif
#ifdef CONFIG_LINK_PTY
vk::platform::PtyTransport pty;
#endif

#ifdef CONFIG_PARAM_STORAGE_FLASH
vk::sim::NorFlash param_flash;
std::array<vk::core::ParameterStorage::Slot, vk::params::count> param_slots{};
vk::core::StaticInstance<vk::core::ParameterStorage> param_storage;

// Loads the stored parameters, before anything reads them, and starts storing changes.
bool start_parameters(const Options& options)
{
    if (options.param_file != nullptr && !param_flash.attach_file(options.param_file)) {
        std::fprintf(stderr, "param-file: cannot open %s\n", options.param_file);
        return false;
    }
    param_storage.emplace(vk::params::parameters(), vk::hal::FlashSectors(param_flash), param_slots,
                          vk::core::work_queue(vk::core::WorkQueueTier::Slow));
    param_storage->load();
    param_storage->start();
    return true;
}
#endif

#ifdef CONFIG_LINK
// The parameter storage, if parameters are stored, for the Link's param_value flags.
const vk::core::ParameterStorage* stored_parameters()
{
#ifdef CONFIG_PARAM_STORAGE_FLASH
    return &*param_storage;
#else
    return nullptr;
#endif
}

// Opens the chosen transport and starts the Link on it; false if it cannot.
bool start_link(const Options& options)
{
    vk::hal::Transport* transport = nullptr;
#ifdef CONFIG_LINK_UDP
    if (options.link == "udp") {
        if (!udp.open(options.udp_address.data(), options.udp_port)) {
            return false;
        }
        std::printf("link transport=udp address=%s:%u\n", options.udp_address.data(), udp.port());
        transport = &udp;
    }
#endif
#ifdef CONFIG_LINK_PTY
    if (options.link == "pty") {
        if (!pty.open()) {
            return false;
        }
        std::printf("link transport=pty path=%s\n", pty.path());
        transport = &pty;
    }
#endif
    if (transport == nullptr) {
        std::fprintf(stderr, "link: transport '%.*s' is not built in\n", static_cast<int>(options.link.size()),
                     options.link.data());
        return false;
    }
    // Tools wait for this line, so it leaves at once.
    std::fflush(stdout);
    host_link.emplace("link0", *transport, vk::link::default_topic_map(),
                      vk::link::Link::Config{.parameters = {.parameters = &vk::params::parameters(),
                                                            .set_hash = vk::params::set_hash,
                                                            .storage = stored_parameters()}},
                      vk::core::work_queue(vk::core::WorkQueueTier::Io));
    host_link->start();
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

#ifdef CONFIG_ESTIMATOR_MAHONY
vk::core::StaticInstance<vk::estimation::AttitudeEstimator> attitude_estimator;

// Right after the IMU on wq:ctrl, so each sample is estimated in the same pass (ADR-0008).
void start_estimator()
{
    attitude_estimator.emplace(vk::estimation::AttitudeEstimator::Config{},
                               vk::core::work_queue(vk::core::WorkQueueTier::Ctrl));
    attitude_estimator->start();
}
#endif

}  // namespace

int main(int argc, char** argv)
{
    Options options;
    if (!parse_options(std::span(argv, static_cast<std::size_t>(argc)), options)) {
        std::fputs("usage: vehkernel-sitl [--duration <seconds>] [--link udp|pty] [--udp <address>:<port>]\n"
                   "                      [--motion still|tilt|rotate|shake] [--param-file <path>]\n",
                   stderr);
        return 2;
    }
    vk::platform::init(options.motion);
#ifdef CONFIG_PARAM_STORAGE_FLASH
    if (!start_parameters(options)) {
        return 1;
    }
#endif
    start_imu();
    vk::platform::start_ground_truth(vk::core::work_queue(vk::core::WorkQueueTier::Ctrl));
#ifdef CONFIG_ESTIMATOR_MAHONY
    start_estimator();
#endif
#ifdef CONFIG_LINK
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
