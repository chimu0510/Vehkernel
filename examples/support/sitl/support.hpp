// SPDX-License-Identifier: GPL-3.0-or-later
// Support code for SITL board examples (spec 0013, ADR-0025).
#pragma once

#include <line.hpp>
#include <vk/core/bus.hpp>
#include <vk/core/critical_section.hpp>
#include <vk/core/parameter_storage.hpp>
#include <vk/core/scheduler.hpp>
#include <vk/core/static_instance.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/drivers/icm20948.hpp>
#include <vk/drivers/ssd1306.hpp>
#include <vk/hal/clock.hpp>
#include <vk/hal/flash.hpp>
#include <vk/hal/i2c.hpp>
#include <vk/hal/transport.hpp>
#include <vk/link/link.hpp>
#include <vk/link/topic_map.hpp>
#include <vk/msg/imu_sample.hpp>
#include <vk/msg/vehicle_attitude_groundtruth.hpp>
#include <vk/params/params.hpp>
#include <vk/platform/platform.hpp>
#include <vk/platform/udp_transport.hpp>
#include <vk/sim/icm20948.hpp>
#include <vk/sim/nor_flash.hpp>
#include <vk/sim/ssd1306.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <numbers>
#include <string_view>
#include <type_traits>

namespace example {

template <typename T>
void report(std::string_view key, const T& value);

namespace detail {

inline vk::core::StaticInstance<vk::drivers::Ssd1306> display_driver;
inline vk::core::StaticInstance<vk::drivers::Icm20948> imu_driver;
inline std::uint64_t stop_duration_us = 0;
inline bool has_stopped = false;

enum class MotionMode : std::uint8_t {
    Default,
    Rotate,
};

inline MotionMode motion_mode = MotionMode::Default;

inline std::array<char, 64> udp_address{"127.0.0.1"};
inline std::uint16_t udp_port = 15600;
inline vk::platform::UdpTransport udp_transport;
inline vk::core::StaticInstance<vk::link::Link> host_link;

inline vk::sim::NorFlash param_flash;
inline std::array<vk::core::ParameterStorage::Slot, vk::params::count> param_slots{};
inline vk::core::StaticInstance<vk::core::ParameterStorage> param_storage;
inline const char* param_file = nullptr;

inline bool parse_address(const char* argument)
{
    const std::string_view text = argument;
    const auto colon = text.rfind(':');
    if (colon == std::string_view::npos || colon == 0) {
        return false;
    }
    const std::string_view address = text.substr(0, colon);
    const char* port = std::next(argument, static_cast<std::ptrdiff_t>(colon + 1));
    if (address.size() >= udp_address.size() || *port == '\0') {
        return false;
    }
    char* end = nullptr;
    const unsigned long value = std::strtoul(port, &end, 10);
    if (*end != '\0' || value > 65535) {
        return false;
    }
    udp_address.fill('\0');
    address.copy(udp_address.data(), address.size());
    udp_port = static_cast<std::uint16_t>(value);
    return true;
}

template <typename MapFn>
vk::link::TopicMap get_topic_map(MapFn& fn, float imu_rate_hz, std::size_t link)
{
    if constexpr (std::is_invocable_v<MapFn, float, std::size_t>) {
        return fn(imu_rate_hz, link);
    } else if constexpr (std::is_invocable_v<MapFn, std::size_t>) {
        return fn(link);
    } else {
        return fn;
    }
}

// Rotating motion source for level rotation about the vertical (z) axis.
// Sweeps a full 360 deg turn over 8 seconds (from 1.0 s to 9.0 s) at 45 deg/s,
// while remaining level (roll = 0, pitch = 0).
class RotatingMotionSource final : public vk::sim::Icm20948::OutputSource {
public:
    static constexpr float rotation_rate = 45.0F * (std::numbers::pi_v<float> / 180.0F);  // 45 deg/s
    static constexpr std::uint64_t start_time_us = 1'000'000;                              // 1.0 s
    static constexpr std::uint64_t rotation_duration_us = 8'000'000;                       // 8.0 s (360 deg)

    struct State {
        float yaw = 0.0F;
        float rate_z = 0.0F;
        std::array<float, 4> q{1.0F, 0.0F, 0.0F, 0.0F};  // [w, x, y, z]
    };

    static State state_at(std::uint64_t time_us)
    {
        State s{};
        if (time_us <= start_time_us) {
            s.yaw = 0.0F;
            s.rate_z = 0.0F;
        } else if (time_us < start_time_us + rotation_duration_us) {
            const float dt = static_cast<float>(time_us - start_time_us) * 1e-6F;
            s.yaw = rotation_rate * dt;
            s.rate_z = rotation_rate;
        } else {
            s.yaw = rotation_rate * 8.0F;  // 2*pi (360 deg)
            s.rate_z = 0.0F;
        }
        const float half_yaw = s.yaw * 0.5F;
        s.q = {std::cos(half_yaw), 0.0F, 0.0F, std::sin(half_yaw)};
        return s;
    }

    void update(vk::sim::Icm20948& imu) override
    {
        const auto s = state_at(vk::hal::SystemClock::now_us());
        // Body (x forward, y right, z down) to sensor (x forward, y left, z up).
        imu.set_gyro({0.0F, 0.0F, -s.rate_z});
        // Specific force upward reaction to gravity in NED -z (sensor +z).
        imu.set_accel({0.0F, 0.0F, 9.80665F});
    }

    std::optional<std::array<float, 3>> field_at(std::uint64_t time_us) override
    {
        const auto s = state_at(time_us);
        // Earth field in NED: (30 uT north, 0 east, 30 uT down).
        // Rotated by q^-1 into body frame:
        const float bx = 30.0F * std::cos(s.yaw);
        const float by = -30.0F * std::sin(s.yaw);
        const float bz = 30.0F;
        // Body to sensor frame: x, -y, -z
        return std::array<float, 3>{bx, -by, -bz};
    }
};

inline RotatingMotionSource rotating_source;

// Work item on wq:ctrl publishing vehicle_attitude_groundtruth for each imu_sample.
class GroundTruthItem final : public vk::core::WorkItem {
public:
    explicit GroundTruthItem(vk::core::WorkQueue& queue)
        : WorkItem("ground_truth", queue)
    {
    }

    void start()
    {
        trigger_on(vk::core::topic<vk::msg::ImuSample>(0));
    }

protected:
    void run() override
    {
        vk::msg::ImuSample sample{};
        while (imu_sub_.copy(sample)) {
            const auto s = RotatingMotionSource::state_at(sample.timestamp);
            vk::msg::VehicleAttitudeGroundtruth truth{};
            truth.timestamp = sample.timestamp;
            truth.q = s.q;
            truth.angular_velocity = {0.0F, 0.0F, s.rate_z};
            publication_.publish(truth);
        }
    }

private:
    vk::core::Subscription<vk::msg::ImuSample> imu_sub_{0};
    vk::core::Publication<vk::msg::VehicleAttitudeGroundtruth> publication_;
};

// Work item on wq:slow reporting true_yaw at the display rate (4 Hz).
class TrueYawItem final : public vk::core::WorkItem {
public:
    static constexpr std::uint32_t interval_us = 250'000;  // 4 Hz

    explicit TrueYawItem(vk::core::WorkQueue& queue)
        : WorkItem("true_yaw", queue)
    {
    }

    void start()
    {
        schedule_periodic(interval_us);
    }

protected:
    void run() override
    {
        vk::msg::VehicleAttitudeGroundtruth truth{};
        while (subscription_.copy(truth)) {
            last_truth_ = truth;
            has_truth_ = true;
        }
        if (has_truth_) {
            // Euler yaw from quaternion [w, x, y, z]:
            const float w = last_truth_.q[0];
            const float x = last_truth_.q[1];
            const float y = last_truth_.q[2];
            const float z = last_truth_.q[3];
            const float yaw_rad = std::atan2(2.0F * (w * z + x * y), 1.0F - 2.0F * (y * y + z * z));
            const float yaw_deg = yaw_rad * (180.0F / std::numbers::pi_v<float>);
            report("true_yaw", yaw_deg);
        }
    }

private:
    vk::core::Subscription<vk::msg::VehicleAttitudeGroundtruth> subscription_{0};
    vk::msg::VehicleAttitudeGroundtruth last_truth_{};
    bool has_truth_ = false;
};

inline vk::core::StaticInstance<GroundTruthItem> ground_truth_item;
inline vk::core::StaticInstance<TrueYawItem> true_yaw_item;

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
        } else if (arg == "--motion" && i + 1 < argc) {
            const std::string_view motion_arg(argv[++i]);
            if (motion_arg == "rotate") {
                detail::motion_mode = detail::MotionMode::Rotate;
            }
        } else if (arg == "--udp" && i + 1 < argc) {
            detail::parse_address(argv[++i]);
        } else if (arg == "--param-file" && i + 1 < argc) {
            detail::param_file = argv[++i];
        }
    }
}

// Starts the FreeRTOS scheduler (ADR-0004, ADR-0018). In SITL, arranges
// stop after duration, runs the scheduler until stopped, reports display_on,
// and exits 0.
inline void start_scheduler()
{
    if (detail::stop_duration_us > 0) {
        vk::platform::stop_after(detail::stop_duration_us);
    }
    vk::core::start_scheduler();
    report("display_on", vk::platform::display().displaying() ? 1 : 0);
    std::exit(0);
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

// Returns the simulated ICM20948 IMU driver on the simulated I2C bus, running on wq:ctrl.
inline vk::drivers::Icm20948& imu()
{
    if (!detail::imu_driver.has_value()) {
        if (detail::motion_mode == detail::MotionMode::Rotate) {
            vk::platform::simulated_imu().set_output_source(&detail::rotating_source);
            detail::ground_truth_item.emplace(vk::core::work_queue(vk::core::WorkQueueTier::Ctrl));
            detail::true_yaw_item.emplace(vk::core::work_queue(vk::core::WorkQueueTier::Slow));
            detail::ground_truth_item->start();
            detail::true_yaw_item->start();
        }
        detail::imu_driver.emplace(
            vk::hal::I2cDevice(vk::platform::imu_bus(), vk::platform::imu_address),
            vk::drivers::Icm20948::Config{
                .instance = 0,
                .board_orientation = vk::hal::Rotation::None,
                .rate_hz = vk::platform::imu_rate_hz},
            vk::core::work_queue(vk::core::WorkQueueTier::Ctrl));
    }
    return *detail::imu_driver;
}

// Loads the stored parameters, from --param-file if given or blank simulated Flash,
// and starts storing changes.
inline void start_parameters()
{
    if (!detail::param_storage.has_value()) {
        if (detail::param_file != nullptr && !detail::param_flash.attach_file(detail::param_file)) {
            std::fprintf(stderr, "param-file: cannot open %s\n", detail::param_file);
        }
        detail::param_storage.emplace(
            vk::params::parameters(), vk::hal::FlashSectors(detail::param_flash), detail::param_slots,
            vk::core::work_queue(vk::core::WorkQueueTier::Slow));
        detail::param_storage->load();
        detail::param_storage->start();
    }
}

// Starts a Link over UDP (default 127.0.0.1:15600, or --udp <address>:<port>),
// printing the bound port line for host test tools.
template <typename MapFn>
inline void start_links(MapFn&& map_fn)
{
    if (!detail::host_link.has_value()) {
        if (!detail::udp_transport.open(detail::udp_address.data(), detail::udp_port)) {
            vk::core::fail("failed to open UDP transport");
        }
        std::printf("link transport=udp address=%s:%u\n", detail::udp_address.data(), detail::udp_transport.port());
        std::fflush(stdout);

        auto map = detail::get_topic_map(map_fn, static_cast<float>(vk::platform::imu_rate_hz), 0);
        detail::host_link.emplace(
            "link0", detail::udp_transport, map,
            vk::link::Link::Config{
                .instance = 0,
                // Parameter access only once start_parameters() has loaded the stored set (09-parameters).
                .parameters = detail::param_storage.has_value()
                        ? vk::link::ParameterAccess{.parameters = &vk::params::parameters(),
                                                    .set_hash = vk::params::set_hash,
                                                    .storage = &*detail::param_storage}
                        : vk::link::ParameterAccess{},
            },
            vk::core::work_queue(vk::core::WorkQueueTier::Io));
        detail::host_link->start();
    }
}

// Returns SITL's Link.
inline vk::link::Link& link(std::size_t /*instance*/ = 0)
{
    return *detail::host_link;
}

}  // namespace example
