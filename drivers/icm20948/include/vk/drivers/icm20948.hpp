// ICM20948 IMU driver (ADR-0009, ADR-0015).
//
// A timer-triggered work item: each run reads one sample of accelerometer,
// gyroscope and temperature over the I2C contract, converts it to SI units,
// rotates it into the body frame (x forward, y right, z down) by the board
// device's mounting rotation, and publishes it on imu_sample. Before the
// first sample it checks WHO_AM_I, resets and configures the device. Read
// failures are counted and retried; a device that keeps failing is probed
// and configured again, so a loose cable never stops the firmware. The
// magnetometer is not used.
#pragma once

#include <vk/core/work_queue.hpp>
#include <vk/hal/i2c.hpp>
#include <vk/hal/rotation.hpp>
#include <vk/msg/imu_sample.hpp>

#include <cstdint>

namespace vk::drivers {

class Icm20948 final : public core::WorkItem {
public:
    struct Config {
        std::uint8_t instance = 0;  // imu_sample instance to publish on
        hal::Rotation rotation = hal::Rotation::None;
        std::uint32_t rate_hz = 200;  // read rate; the output data rate is set at or above it
    };

    enum class State : std::uint8_t {
        Probing,      // waiting for WHO_AM_I to answer 0xEA
        Resetting,    // device reset issued, waiting for it to restart
        Running,      // configured, publishing samples
    };

    struct Status {
        State state;
        std::uint32_t samples;         // published since boot
        std::uint32_t read_failures;   // sample reads that failed
        std::uint32_t probe_failures;  // probes without the expected WHO_AM_I
        std::uint8_t last_who_am_i;    // last value read, 0 if the read failed
    };

    static constexpr std::uint8_t who_am_i_value = 0xEA;

    Icm20948(const hal::I2cDevice& device, const Config& config, core::WorkQueue& queue);

    // Starts the periodic reads at the configured rate.
    void start();

    [[nodiscard]] Status status() const;

protected:
    void run() override;

private:
    void probe(std::uint64_t now);
    void configure(std::uint64_t now);
    void read_sample(std::uint64_t now);
    bool select_bank(std::uint8_t bank);

    hal::I2cDevice device_;
    Config config_;
    core::Publication<msg::ImuSample> publication_;

    State state_ = State::Probing;
    std::uint64_t next_probe_us_ = 0;
    std::uint64_t reset_at_us_ = 0;
    std::uint32_t consecutive_failures_ = 0;
    std::uint32_t samples_ = 0;
    std::uint32_t read_failures_ = 0;
    std::uint32_t probe_failures_ = 0;
    std::uint8_t last_who_am_i_ = 0;
};

}  // namespace vk::drivers
