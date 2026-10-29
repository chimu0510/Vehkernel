// SPDX-License-Identifier: GPL-3.0-or-later
// ICM20948 IMU driver (ADR-0009, ADR-0015).
//
// A timer-triggered work item: each run reads one sample of accelerometer,
// gyroscope and temperature over the I2C contract, converts it to SI units,
// rotates it into the body frame (x forward, y right, z down) by the board
// device's mounting rotation and then the board orientation, composed once
// at construction, and publishes it on imu_sample. Before the
// first sample it checks WHO_AM_I, resets and configures the device. Read
// failures are counted and retried; a device that keeps failing is probed
// and configured again, so a loose cable never stops the firmware. When
// DRIVER_ICM20948_MAG is enabled, it also enables the ICM20948's I2C master,
// probes and configures the internal AK09916 magnetometer via non-blocking
// slave 4 transfers, and reads samples through the FIFO (spec 0012).
#pragma once

#include <vk/config.h>
#include <vk/core/work_queue.hpp>
#include <vk/hal/i2c.hpp>
#include <vk/hal/rotation.hpp>
#include <vk/msg/imu_sample.hpp>

#if defined(CONFIG_DRIVER_ICM20948_MAG)
#include <vk/msg/magnetometer_sample.hpp>
#endif

#include <cstdint>

namespace vk::drivers {

class Icm20948 final : public core::WorkItem {
public:
    struct Config {
        std::uint8_t instance = 0;  // imu_sample instance to publish on
        hal::Rotation rotation = hal::Rotation::None;           // mounting rotation on the board
        hal::Rotation board_orientation = hal::Rotation::None;  // the board in the vehicle, SENS_BOARD_ROT
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
#if defined(CONFIG_DRIVER_ICM20948_MAG)
        std::uint32_t mag_samples = 0;
        std::uint32_t mag_failures = 0;
        std::uint32_t mag_overflows = 0;
        bool mag_configured = false;
#endif
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

#if defined(CONFIG_DRIVER_ICM20948_MAG)
    enum class MagState : std::uint8_t {
        Probing,        // waiting for probe interval to start setup
        WaitingWhoAmI,  // slave 4 WHO_AM_I read started, waiting for completion
        WaitingCntl2,   // slave 4 CNTL2 write started, waiting for completion
        Running,        // slave 0 & FIFO enabled, reading FIFO records
    };

    void service_magnetometer(std::uint64_t now);
    void read_mag_fifo(std::uint64_t now);
    void reset_mag_fifo();
    void reset_mag_setup(std::uint64_t now);
#endif

    hal::I2cDevice device_;
    Config config_;
    hal::RotationMatrix to_body_;  // sensor frame to body frame
    core::Publication<msg::ImuSample> publication_;
#if defined(CONFIG_DRIVER_ICM20948_MAG)
    core::Publication<msg::MagnetometerSample> mag_publication_;
    MagState mag_state_ = MagState::Probing;
    std::uint64_t next_mag_probe_us_ = 0;
    std::uint64_t mag_step_started_us_ = 0;
    std::uint64_t last_mag_sample_us_ = 0;
    std::uint32_t mag_samples_ = 0;
    std::uint32_t mag_failures_ = 0;
    std::uint32_t mag_overflows_ = 0;
#endif

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
