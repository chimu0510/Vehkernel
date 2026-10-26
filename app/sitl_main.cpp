// SPDX-License-Identifier: GPL-3.0-or-later
// SITL entry point: the firmware on the FreeRTOS POSIX port (ADR-0018).
//
// Usage: vehkernel-sitl [--duration <seconds>] [--link udp|pty] [--udp <address>:<port>]
//                       [--motion vehicle|still|tilt|rotate|shake] [--slip <fraction>]
//                       [--param-file <path>]
//                       [--actuator-power-off] [--reversed-motor <output>]
//                       [--display <path>] [--reset-cause <cause>[:<fault>]]
//                       [--fault assert|fail]
//
// The run stops by itself after the duration (default 1 s; 0 runs until
// interrupted) and prints a report: one line per topic instance and per work
// item, as space-separated key=value pairs, so scripts can parse it. A Link
// streams topics to the development host, over UDP by default
// (127.0.0.1:15600, port 0 for any free port) or over a pty with --link pty.
// Its line, with the port bound or the pty's path, is printed when it starts.
// By default the simulated board sits in a ground vehicle (ADR-0018) that its
// motors move, described by the same parameters as the firmware's vehicle
// model, read at boot; --slip makes it lose that fraction of its wheels'
// motion. The IMU feels the vehicle's turns and accelerations, and its true
// pose goes out on vehicle_local_odometry_groundtruth. With --motion and a
// scripted profile, the IMU follows that instead and the vehicle is not
// simulated. Either way the true attitude is published on
// vehicle_attitude_groundtruth.
// Parameters are stored in a simulated NOR Flash: blank at every start, so
// runs are reproducible, or kept in the file given with --param-file.
// The simulated board senses actuator power, on unless --actuator-power-off
// cuts it at start; the Link's sim_actuator_power frame cuts or restores it.
// Its four motors can be wired the wrong way round with --reversed-motor,
// once per output, for the bring-up case.
// Its display is a simulated SSD1306; --display writes what it shows to a
// PBM file after every change (ADR-0022). Its user key is pressed and
// released by the Link's sim_user_key frame, and its gamepad, disconnected
// at start, is set by the sim_gamepad frame (spec 0010).
// A reboot request over the Link ends the run at once, without the report,
// with exit status 10 to start the firmware again or 11 for the bootloader
// (ADR-0023). The fault stop ends it with exit status 12 (ADR-0005); --fault
// raises one once an output has been driven for 0.5 s. A run starts as if
// powered on; whoever starts it again after a reset gives the reset cause
// with --reset-cause, such as watchdog or fault:assert, as the board would
// read it at boot.
#include <vk/config.h>

#ifdef CONFIG_ACTUATOR_CONTROL
#include <vk/actuators/actuator_task.hpp>
#endif
#include <vk/commander/commander_task.hpp>
#include <vk/remote_control/remote_control_task.hpp>
#include <vk/core/bus.hpp>
#include <vk/core/fault.hpp>
#include <vk/core/scheduler.hpp>
#include <vk/core/static_instance.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/drivers/icm20948.hpp>
#ifdef CONFIG_ESTIMATOR_MAHONY
#include <vk/estimation/attitude_estimator.hpp>
#endif
#include <vk/hal/i2c.hpp>
#ifdef CONFIG_LOCAL_DISPLAY
#include <vk/drivers/ssd1306.hpp>
#include <vk/hal/display_bus.hpp>
#include <vk/local_display/local_display.hpp>
#endif
#ifdef CONFIG_LOCAL_MENU
#include <vk/local_display/local_menu_task.hpp>
#endif
#include <vk/msg/enums.hpp>
#include <vk/msg/registry.hpp>
#include <vk/params/params.hpp>
#ifdef CONFIG_PARAM_STORAGE_FLASH
#include <vk/core/parameter_storage.hpp>
#include <vk/hal/flash.hpp>
#include <vk/sim/nor_flash.hpp>
#endif
#include <vk/platform/platform.hpp>
#include <vk/vehicle_model/vehicle_description.hpp>

#ifdef CONFIG_CONTROL_CHAIN
#include <vk/control/control_chain_task.hpp>
#endif
#ifdef CONFIG_ESTIMATOR_LOCAL_ODOMETRY
#include <vk/estimation/local_odometry.hpp>
#endif
#ifdef CONFIG_ACTUATOR_CONTROL
#include <vk/vehicle_model/velocity_measurement_task.hpp>
#endif

#ifdef CONFIG_LINK
#include <vk/hal/transport.hpp>
#include <vk/link/link.hpp>
#include <vk/link/sim_handler.hpp>
#include <vk/link/topic_map.hpp>
#endif
#ifdef CONFIG_LINK_UDP
#include <vk/platform/udp_transport.hpp>
#endif
#ifdef CONFIG_LINK_PTY
#include <vk/platform/pty_transport.hpp>
#endif

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iterator>
#include <optional>
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
    std::optional<vk::sim::MotionProfile> motion;  // none: the ground vehicle
    float slip = 0.0F;
    const char* param_file = nullptr;  // nullptr: blank Flash in memory
    bool actuator_power = true;
    std::array<bool, vk::platform::actuator_count> reversed_motors{};
    const char* display_file = nullptr;  // nullptr: the display's image stays in memory
    vk::core::ResetReport reset{.cause = vk::core::ResetCause::PowerOn};
    vk::core::Fault fault = vk::core::Fault::None;  // --fault
};

// The value of `Enum` whose schema name is `text` in lower case.
template <class Enum>
std::optional<Enum> enum_from_name(std::string_view text)
{
    for (const auto& entry : vk::msg::EnumValues<Enum>::values) {
        if (entry.name.size() == text.size() &&
            std::equal(text.begin(), text.end(), entry.name.begin(),
                       [](char a, char b) { return std::toupper(static_cast<unsigned char>(a)) == b; })) {
            return entry.value;
        }
    }
    return std::nullopt;
}

// <cause>[:<fault>], as power_on or fault:assert; a fault only with FAULT.
bool parse_reset_cause(std::string_view text, Options& options)
{
    const auto colon = text.find(':');
    const auto cause = enum_from_name<vk::core::ResetCause>(text.substr(0, colon));
    if (!cause) {
        return false;
    }
    options.reset = {.cause = *cause};
    if (colon == std::string_view::npos) {
        return true;
    }
    const auto fault = enum_from_name<vk::core::Fault>(text.substr(colon + 1));
    if (*cause != vk::core::ResetCause::Fault || !fault) {
        return false;
    }
    options.reset.fault = *fault;
    return true;
}

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
        if (arg == "--actuator-power-off") {
            options.actuator_power = false;
            continue;
        }
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
            if (std::string_view(value) == "vehicle") {
                options.motion.reset();
                continue;
            }
            const auto profile = vk::sim::motion_profile(value);
            if (!profile) {
                return false;
            }
            options.motion = profile;
        } else if (arg == "--slip") {
            char* end = nullptr;
            options.slip = std::strtof(value, &end);
            if (*end != '\0' || !(options.slip >= 0.0F && options.slip < 1.0F)) {
                return false;
            }
        } else if (arg == "--udp") {
            if (!parse_address(value, options)) {
                return false;
            }
        } else if (arg == "--param-file") {
            options.param_file = value;
        } else if (arg == "--display") {
            options.display_file = value;
        } else if (arg == "--reset-cause") {
            if (!parse_reset_cause(value, options)) {
                return false;
            }
        } else if (arg == "--fault") {
            const auto fault = enum_from_name<vk::core::Fault>(value);
            if (!fault || (*fault != vk::core::Fault::Assert && *fault != vk::core::Fault::Fail)) {
                return false;
            }
            options.fault = *fault;
        } else if (arg == "--reversed-motor") {
            char* end = nullptr;
            const unsigned long output = std::strtoul(value, &end, 10);
            if (*end != '\0' || output < 1 || output > vk::platform::actuator_count) {
                return false;
            }
            options.reversed_motors.at(output - 1) = true;
        } else {
            return false;
        }
    }
    return true;
}

// The ground vehicle, from the vehicle description the firmware reads: the
// same frame, geometry and output mapping. A description no model can drive
// leaves the board still, as the scripted still motion.
void start_vehicle(const Options& options)
{
#ifdef CONFIG_VEHICLE_MODEL_WHEELED
    const vk::vehicle_model::VehicleDescription description = vk::vehicle_model::description_from_parameters();
    static vk::vehicle_model::WheeledModel model;
    model.set(description.wheeled);
    if (description.vehicle_type != vk::msg::VehicleType::Ground ||
        description.frame_class != vk::msg::FrameClass::Wheeled || !model.valid()) {
        return;
    }
    std::array<std::size_t, vk::estimation::max_wheels> outputs{};
    for (std::size_t i = 0; i < model.kinematics().size(); ++i) {
        outputs.at(i) = model.output(i);
    }
    vk::platform::start_vehicle(model.kinematics(), outputs, options.slip);
#else
    static_cast<void>(options);
#endif
}

vk::core::StaticInstance<vk::commander::CommanderTask> commander;

constexpr std::uint32_t control_interval_us = 1'000'000 / vk::platform::imu_rate_hz;

// The commander on wq:ctrl at the IMU's rate (ADR-0005). SITL's board has no
// revision-ID pins, so it is verified once the simulated IMU answers.
void start_commander()
{
#ifdef CONFIG_ACTUATOR_CONTROL
    constexpr bool actuators = true;
#else
    constexpr bool actuators = false;
#endif
#ifdef CONFIG_BRINGUP_MOTORS
    constexpr bool bringup = true;
#else
    constexpr bool bringup = false;
#endif
    commander.emplace(vk::commander::CommanderTask::Config{.interval_us = control_interval_us,
                                                           .actuator_power = &vk::platform::actuator_power,
                                                           .actuators_started = actuators,
                                                           .bringup = bringup,
                                                           .output_watchdog_trips = &vk::platform::output_watchdog_trips,
                                                           .reset = &vk::platform::reset,
                                                           .reset_report = vk::platform::reset_report()},
                      vk::core::work_queue(vk::core::WorkQueueTier::Ctrl));
    commander->start();
}

vk::core::StaticInstance<vk::remote_control::RemoteControlTask> remote_control;

// The remote control on wq:ctrl after the commander (spec 0010).
void start_remote_control()
{
    remote_control.emplace(vk::remote_control::RemoteControlTask::Config{.interval_us = control_interval_us},
                           vk::core::work_queue(vk::core::WorkQueueTier::Ctrl));
    remote_control->start();
}


#ifdef CONFIG_ACTUATOR_CONTROL
vk::core::StaticInstance<vk::actuators::ActuatorTask> actuator_control;

vk::vehicle_model::VehicleModels vehicle_models;
const vk::vehicle_model::VehicleModel* vehicle_model = nullptr;
vk::core::StaticInstance<vk::vehicle_model::VelocityMeasurementTask> velocity_measurement;

// The vehicle model the parameters select at boot (ADR-0013), and its body
// velocity measurement from the wheel encoders on wq:ctrl (ADR-0014).
void start_vehicle_model()
{
    vehicle_model = vehicle_models.select(vk::vehicle_model::description_from_parameters());
    velocity_measurement.emplace(vk::vehicle_model::VelocityMeasurementTask::Config{.model = vehicle_model},
                                 vk::core::work_queue(vk::core::WorkQueueTier::Ctrl));
    velocity_measurement->start();
}

#ifdef CONFIG_CONTROL_CHAIN
vk::core::StaticInstance<vk::control::ControlChainTask> control_chain;

// The control chain on wq:ctrl before actuator control, over the vehicle
// model selected at boot (ADR-0013).
void start_control_chain()
{
    control_chain.emplace(
        vk::control::ControlChainTask::Config{.interval_us = control_interval_us, .model = vehicle_model},
        vk::core::work_queue(vk::core::WorkQueueTier::Ctrl));
    control_chain->start();
}
#endif

#ifdef CONFIG_ESTIMATOR_LOCAL_ODOMETRY
vk::core::StaticInstance<vk::estimation::LocalOdometry> local_odometry;

// Local odometry on wq:ctrl, from the wheels' body velocity measurement (ADR-0014).
void start_local_odometry()
{
    local_odometry.emplace(vk::estimation::LocalOdometry::Config{}, vk::core::work_queue(vk::core::WorkQueueTier::Ctrl));
    local_odometry->start();
}
#endif

// Actuator control on wq:ctrl after the commander, over the simulated motors (spec 0006).
void start_actuators()
{
#ifdef CONFIG_BRINGUP_MOTORS
    constexpr bool bringup = true;
#else
    constexpr bool bringup = false;
#endif
    actuator_control.emplace(vk::actuators::ActuatorTask::Config{.interval_us = control_interval_us,
                                                                 .channels = vk::platform::motor_channels(),
                                                                 .bringup = bringup},
                             vk::core::work_queue(vk::core::WorkQueueTier::Ctrl));
    actuator_control->start();
    vk::platform::start_output_watchdog(control_interval_us);
}
#endif

#ifdef CONFIG_LOCAL_DISPLAY
vk::core::StaticInstance<vk::drivers::Ssd1306> display_driver;
vk::core::StaticInstance<vk::local_display::LocalDisplay> local_display;
#ifdef CONFIG_LOCAL_MENU
vk::core::StaticInstance<vk::local_display::LocalMenuTask> local_menu;
#endif

// The real driver against the simulated SSD1306, and the local menu on the
// simulated user key, both on wq:slow (ADR-0022).
void start_local_display(const Options& options)
{
    if (options.display_file != nullptr) {
        vk::platform::display().write_image_to(options.display_file);
    }
    const vk::local_display::LocalMenu* menu = nullptr;
#ifdef CONFIG_LOCAL_MENU
    local_menu.emplace(vk::local_display::LocalMenuTask::Config{.key_pressed = &vk::platform::user_key_pressed},
                       vk::params::parameters(), vk::params::menu(), vk::core::work_queue(vk::core::WorkQueueTier::Slow));
    local_menu->start();
    menu = &local_menu->menu();
#endif
    display_driver.emplace(vk::hal::DisplayDevice(vk::platform::display()));
    local_display.emplace(*display_driver, menu, vk::core::work_queue(vk::core::WorkQueueTier::Slow));
    local_display->start();
}
#endif

#ifdef CONFIG_LINK
vk::core::StaticInstance<vk::link::Link> host_link;
// The Link's sim_actuator_power, sim_user_key and sim_gamepad frames act on the simulator.
vk::link::SimHandler sim_frames({.set_actuator_power = &vk::platform::set_actuator_power,
                                 .set_user_key = &vk::platform::set_user_key,
                                 .set_gamepad = &vk::platform::set_gamepad});
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
    host_link.emplace("link0", *transport, vk::link::default_topic_map(static_cast<float>(vk::platform::imu_rate_hz)),
                      vk::link::Link::Config{.parameters = {.parameters = &vk::params::parameters(),
                                                            .set_hash = vk::params::set_hash,
                                                            .storage = stored_parameters()},
                                             .control = {.enabled = true},
                                             .sim_frames = &sim_frames},
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
                   "                      [--motion vehicle|still|tilt|rotate|shake] [--slip <fraction>]\n"
                   "                      [--param-file <path>]\n"
                   "                      [--actuator-power-off] [--reversed-motor <output>]\n"
                   "                      [--display <path>] [--reset-cause <cause>[:<fault>]]\n"
                   "                      [--fault assert|fail]\n",
                   stderr);
        return 2;
    }
    vk::platform::init(options.motion.value_or(vk::sim::MotionProfile::Still));
    vk::platform::set_reset_report(options.reset);
    vk::platform::raise_fault_when_driven(options.fault);
    vk::platform::set_actuator_power(options.actuator_power);
    for (std::size_t output = 1; output <= vk::platform::actuator_count; ++output) {
        vk::platform::motor(output).set_reversed(options.reversed_motors.at(output - 1));
    }
#ifdef CONFIG_PARAM_STORAGE_FLASH
    if (!start_parameters(options)) {
        return 1;
    }
#endif
    if (!options.motion) {
        start_vehicle(options);
    }
    start_imu();
    vk::platform::start_ground_truth(vk::core::work_queue(vk::core::WorkQueueTier::Ctrl));
#ifdef CONFIG_ESTIMATOR_MAHONY
    start_estimator();
#endif
    start_commander();
    vk::platform::start_sim_gamepad(vk::core::work_queue(vk::core::WorkQueueTier::Ctrl));
    start_remote_control();
#ifdef CONFIG_ACTUATOR_CONTROL
    start_vehicle_model();
#ifdef CONFIG_CONTROL_CHAIN
    start_control_chain();
#endif
#ifdef CONFIG_ESTIMATOR_LOCAL_ODOMETRY
    start_local_odometry();
#endif
    start_actuators();
#endif
#ifdef CONFIG_LOCAL_DISPLAY
    start_local_display(options);
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
