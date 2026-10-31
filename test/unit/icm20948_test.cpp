// SPDX-License-Identifier: GPL-3.0-or-later
// ICM20948 driver against the device's register model on the simulated
// peripheral bus (ADR-0018 L1). The driver's work queue runs on a fake clock.
#include <vk/config.h>
#include <vk/core/bus.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/drivers/icm20948.hpp>
#include <vk/hal/i2c.hpp>
#include <vk/msg/imu_sample.hpp>
#include <vk/msg/magnetometer_sample.hpp>
#include <vk/params/params.hpp>
#include <vk/sim/i2c_bus.hpp>
#include <vk/sim/icm20948.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <numbers>
#include <string_view>
#include <vector>

namespace {

using vk::drivers::Icm20948;
using State = Icm20948::State;

std::uint64_t fake_now = 0;

std::uint64_t fake_clock() noexcept
{
    return fake_now;
}

// Any other device answering at the IMU's address.
class WrongDevice final : public vk::sim::RegisterDevice {
protected:
    void write_register(std::uint8_t /*reg*/, std::uint8_t /*value*/) override {}
    std::uint8_t read_register(std::uint8_t /*reg*/) override { return 0x12; }
};

constexpr std::uint32_t rate_hz = 200;
constexpr std::uint64_t period_us = 1'000'000 / rate_hz;
constexpr float accel_lsb = 9.80665F / 2048.0F;
constexpr float gyro_lsb = std::numbers::pi_v<float> / 180.0F / 16.4F;
constexpr float mag_lsb = 0.15F;

class Icm20948Test : public ::testing::Test {
protected:
    void SetUp() override
    {
        fake_now = 1'000'000;
        bus.attach(vk::sim::Icm20948::default_address, model);

        vk::params::cal.mag_xoff = 0.0F;
        vk::params::cal.mag_yoff = 0.0F;
        vk::params::cal.mag_zoff = 0.0F;
        vk::params::cal.mag_xscale = 1.0F;
        vk::params::cal.mag_yscale = 1.0F;
        vk::params::cal.mag_zscale = 1.0F;
    }

    // Runs the driver's work queue for `duration_us`, one period at a time.
    void run_for(std::uint64_t duration_us)
    {
        for (std::uint64_t t = 0; t < duration_us; t += period_us) {
            fake_now += period_us;
            queue.process();
        }
    }

    // Drains `sub`, returning the newest sample, and how many there were.
    static int drain(vk::core::Subscription<vk::msg::ImuSample>& sub, vk::msg::ImuSample& newest)
    {
        int count = 0;
        while (sub.copy(newest)) {
            ++count;
        }
        return count;
    }

    static int drain_mag(vk::core::Subscription<vk::msg::MagnetometerSample>& sub,
                         vk::msg::MagnetometerSample& newest)
    {
        int count = 0;
        while (sub.copy(newest)) {
            ++count;
        }
        return count;
    }

    vk::sim::I2cBus bus;
    vk::sim::Icm20948 model{&fake_clock};
    vk::core::WorkQueue queue{"wq:test", 1, &fake_clock};
};

TEST_F(Icm20948Test, WrongWhoAmIIsNeverConfigured)
{
    WrongDevice wrong;
    bus.attach(vk::sim::Icm20948::default_address, wrong);
    Icm20948 imu(vk::hal::I2cDevice(bus, 0x68), {.instance = 1, .rate_hz = rate_hz}, queue);
    imu.start();
    vk::core::Subscription<vk::msg::ImuSample> sub(1);
    run_for(1'000'000);
    const auto status = imu.status();
    EXPECT_EQ(status.state, State::Probing);
    EXPECT_EQ(status.last_who_am_i, 0x12);
    EXPECT_GE(status.probe_failures, 5U);  // retried, about every 100 ms
    EXPECT_FALSE(sub.updated());
}

TEST_F(Icm20948Test, MissingDeviceKeepsProbing)
{
    bus.detach(vk::sim::Icm20948::default_address);
    Icm20948 imu(vk::hal::I2cDevice(bus, 0x68), {.rate_hz = rate_hz}, queue);
    imu.start();
    run_for(500'000);
    EXPECT_EQ(imu.status().state, State::Probing);
    EXPECT_EQ(imu.status().last_who_am_i, 0);
    EXPECT_GT(imu.status().probe_failures, 0U);
}

TEST_F(Icm20948Test, ResetsAndConfiguresTheDevice)
{
    Icm20948 imu(vk::hal::I2cDevice(bus, 0x68), {.rate_hz = rate_hz}, queue);
    imu.start();
    run_for(300'000);
    EXPECT_EQ(imu.status().state, State::Running);
    EXPECT_EQ(model.resets(), 1);
    EXPECT_FALSE(model.sleeping());
    EXPECT_EQ(model.register_value(0, 0x07), 0x00);  // PWR_MGMT_2: every axis on
    EXPECT_EQ(model.register_value(2, 0x01), 0x1F);  // GYRO_CONFIG_1: +-2000 dps, DLPF 3
    EXPECT_EQ(model.register_value(2, 0x14), 0x1F);  // ACCEL_CONFIG: +-16 g, DLPF 3
    // 1125 Hz / (1 + 4) = 225 Hz, the lowest output data rate at or above 200 Hz.
    EXPECT_EQ(model.register_value(2, 0x00), 4);
    EXPECT_EQ(model.register_value(2, 0x11), 4);
    EXPECT_EQ(model.register_value(0, 0x7F), 0x00);  // back in bank 0
}

TEST_F(Icm20948Test, PublishesOneSamplePerPeriodWithItsTimestamp)
{
    Icm20948 imu(vk::hal::I2cDevice(bus, 0x68), {.instance = 0, .rate_hz = rate_hz}, queue);
    imu.start();
    vk::core::Subscription<vk::msg::ImuSample> sub(0);
    run_for(300'000);
    vk::msg::ImuSample sample{};
    drain(sub, sample);
    const auto missed_before = sub.missed();
    for (int i = 0; i < 10; ++i) {
        run_for(period_us);
        EXPECT_EQ(drain(sub, sample), 1);
        EXPECT_EQ(sample.timestamp, fake_now);
    }
    EXPECT_EQ(sample.device_instance, 0);
    EXPECT_EQ(sub.missed(), missed_before);
}

TEST_F(Icm20948Test, ConvertsToSiUnitsInTheBodyFrame)
{
    // Lying flat, turning: the sensor frame is z up, the body frame z down.
    model.set_accel({1.0F, -2.0F, 9.80665F});
    model.set_gyro({0.1F, -0.2F, 0.3F});
    model.set_temperature(30.0F);
    Icm20948 imu(vk::hal::I2cDevice(bus, 0x68), {.rate_hz = rate_hz}, queue);
    imu.start();
    vk::core::Subscription<vk::msg::ImuSample> sub(0);
    run_for(300'000);
    vk::msg::ImuSample sample{};
    ASSERT_GT(drain(sub, sample), 0);
    EXPECT_NEAR(sample.accel[0], 1.0F, accel_lsb);
    EXPECT_NEAR(sample.accel[1], 2.0F, accel_lsb);
    EXPECT_NEAR(sample.accel[2], -9.80665F, accel_lsb);
    EXPECT_NEAR(sample.gyro[0], 0.1F, gyro_lsb);
    EXPECT_NEAR(sample.gyro[1], 0.2F, gyro_lsb);
    EXPECT_NEAR(sample.gyro[2], -0.3F, gyro_lsb);
    EXPECT_NEAR(sample.temperature, 30.0F, 0.01F);
}

TEST_F(Icm20948Test, AppliesTheMountingRotation)
{
    model.set_accel({1.0F, 0.0F, 0.0F});
    model.set_gyro({0.0F, 0.0F, 0.0F});
    Icm20948 imu(vk::hal::I2cDevice(bus, 0x68),
                 {.instance = 1, .rotation = vk::hal::Rotation::Yaw90, .rate_hz = rate_hz}, queue);
    imu.start();
    vk::core::Subscription<vk::msg::ImuSample> sub(1);
    run_for(300'000);
    vk::msg::ImuSample sample{};
    ASSERT_GT(drain(sub, sample), 0);
    EXPECT_NEAR(sample.accel[0], 0.0F, accel_lsb);
    EXPECT_NEAR(sample.accel[1], 1.0F, accel_lsb);
    EXPECT_EQ(sample.device_instance, 1);
}

TEST_F(Icm20948Test, AppliesTheMountingRotationThenTheBoardOrientation)
{
    // Sensor x is body x before any rotation; sensor z (up) is body -z.
    model.set_accel({1.0F, 0.0F, 0.0F});
    model.set_gyro({0.0F, 0.0F, 0.3F});
    Icm20948 imu(vk::hal::I2cDevice(bus, 0x68),
                 {.instance = 1,
                  .rotation = vk::hal::Rotation::Yaw90,
                  .board_orientation = vk::hal::Rotation::Roll180,
                  .rate_hz = rate_hz},
                 queue);
    imu.start();
    vk::core::Subscription<vk::msg::ImuSample> sub(1);
    run_for(300'000);
    vk::msg::ImuSample sample{};
    ASSERT_GT(drain(sub, sample), 0);
    // x -> yaw 90 -> y -> roll 180 -> -y
    EXPECT_NEAR(sample.accel[0], 0.0F, accel_lsb);
    EXPECT_NEAR(sample.accel[1], -1.0F, accel_lsb);
    EXPECT_NEAR(sample.accel[2], 0.0F, accel_lsb);
    // -z -> yaw 90 -> -z -> roll 180 -> z
    EXPECT_NEAR(sample.gyro[2], 0.3F, gyro_lsb);
}

TEST_F(Icm20948Test, CountsReadFailuresAndRecoversWhenTheDeviceReturns)
{
    Icm20948 imu(vk::hal::I2cDevice(bus, 0x68), {.rate_hz = rate_hz}, queue);
    imu.start();
    vk::core::Subscription<vk::msg::ImuSample> sub(0);
    run_for(300'000);
    ASSERT_EQ(imu.status().state, State::Running);

    model.set_responding(false);  // A loose cable.
    run_for(500'000);
    EXPECT_GT(imu.status().read_failures, 0U);
    EXPECT_NE(imu.status().state, State::Running);
    vk::msg::ImuSample sample{};
    drain(sub, sample);

    model.set_responding(true);
    run_for(500'000);
    EXPECT_EQ(imu.status().state, State::Running);
    EXPECT_GT(drain(sub, sample), 0);
    EXPECT_EQ(model.resets(), 2);  // Probed and configured again.
}

#if defined(CONFIG_DRIVER_ICM20948_MAG)
// The setup sequence runs one non-blocking step per periodic run.
TEST_F(Icm20948Test, SetupSequenceCheckedStepByStepWithoutBlocking)
{
    Icm20948 imu(vk::hal::I2cDevice(bus, 0x68), {.rate_hz = rate_hz}, queue);
    imu.start();

    // Advance through initial probe and reset until the WHO_AM_I read through Slave 4 starts.
    int steps = 0;
    while ((model.register_value(3, 0x13) != 0x8C || model.register_value(3, 0x14) != 0x01) && steps < 50) {
        run_for(period_us);
        ++steps;
    }
    ASSERT_LT(steps, 50);
    EXPECT_EQ(imu.status().state, State::Running);
    EXPECT_FALSE(imu.status().mag_configured);
    EXPECT_EQ(model.register_value(0, 0x05), 0x00);  // LP_CONFIG: I2C master not duty-cycled
    EXPECT_EQ(model.register_value(0, 0x03), 0x20);  // USER_CTRL: I2C_MST_EN
    EXPECT_EQ(model.register_value(3, 0x01), 0x07);  // I2C_MST_CTRL: 400 kHz clock
    EXPECT_EQ(model.register_value(3, 0x13), 0x8C);  // Slave 4 addr: AK09916 read
    EXPECT_EQ(model.register_value(3, 0x14), 0x01);  // Slave 4 reg: WIA2 / WHO_AM_I

    // Step 2: Next run checks I2C_MST_STATUS, sees done, reads WHO_AM_I, and starts CNTL2 write.
    run_for(period_us);
    EXPECT_FALSE(imu.status().mag_configured);
    EXPECT_EQ(model.register_value(3, 0x13), 0x0C);  // Slave 4 addr: AK09916 write
    EXPECT_EQ(model.register_value(3, 0x14), 0x31);  // Slave 4 reg: CNTL2
    EXPECT_EQ(model.register_value(3, 0x16), 0x08);  // Slave 4 DO: continuous mode 4

    // Step 3: Next run sees CNTL2 done, configures Slave 0 and FIFO, and transitions to Running.
    run_for(period_us);
    EXPECT_TRUE(imu.status().mag_configured);
    EXPECT_EQ(model.register_value(3, 0x03), 0x8C);  // Slave 0 addr: AK09916 read
    EXPECT_EQ(model.register_value(3, 0x04), 0x10);  // Slave 0 reg: ST1
    EXPECT_EQ(model.register_value(3, 0x05), 0x89);  // Slave 0 ctrl: enable | 9 bytes
    EXPECT_EQ(model.ak09916_register_value(0x31), 0x08);  // AK09916 in mode 4
    EXPECT_EQ(model.register_value(0, 0x66), 0x01);  // FIFO_EN_1: SLV_0_FIFO_EN
    EXPECT_EQ(model.register_value(0, 0x03), 0x60);  // USER_CTRL: FIFO_EN | I2C_MST_EN
    EXPECT_EQ(model.register_value(0, 0x7F), 0x00);  // Left in Bank 0
}

// Every sample is published once at 200 Hz read rate and 225 Hz master cycle rate.
TEST_F(Icm20948Test, EverySamplePublishedExactlyOnceOverOneSecond)
{
    Icm20948 imu(vk::hal::I2cDevice(bus, 0x68), {.rate_hz = rate_hz}, queue);
    imu.start();
    vk::core::Subscription<vk::msg::ImuSample> imu_sub(0);
    vk::core::Subscription<vk::msg::MagnetometerSample> mag_sub(0);
    run_for(300'000);
    EXPECT_TRUE(imu.status().mag_configured);

    vk::msg::ImuSample imu_sample{};
    vk::msg::MagnetometerSample mag_sample{};
    drain(imu_sub, imu_sample);
    drain_mag(mag_sub, mag_sample);

    std::vector<float> received_x;
    received_x.reserve(100);
    int imu_count = 0;

    // Run for 1.0 simulated second (200 periods of 5 ms at 200 Hz).
    for (int i = 0; i < 200; ++i) {
        // Change field with time so each 10 ms AK09916 sample captures a unique value.
        const float field_x = 10.0F + static_cast<float>(i) * 0.30F;
        model.set_field({field_x, 0.0F, 0.0F});
        run_for(period_us);
        if (drain(imu_sub, imu_sample) > 0) {
            ++imu_count;
        }
        while (mag_sub.copy(mag_sample)) {
            received_x.push_back(mag_sample.field[0]);
        }
    }

    // IMU runs at 200 Hz (200 samples in 1 s).
    EXPECT_EQ(imu_count, 200);
    // AK09916 produces exactly 100 samples in 1 s at 100 Hz.
    EXPECT_EQ(received_x.size(), 100U);

    // Verify every sample is unique and strictly increasing (no duplicates, no missed samples).
    for (std::size_t i = 1; i < received_x.size(); ++i) {
        EXPECT_GT(received_x[i], received_x[i - 1]);
    }
}

// A bad FIFO count triggers a reset of the FIFO and increments the failure counter.
TEST_F(Icm20948Test, BadFifoCountResetsFifoAndRecovers)
{
    Icm20948 imu(vk::hal::I2cDevice(bus, 0x68), {.rate_hz = rate_hz}, queue);
    imu.start();
    vk::core::Subscription<vk::msg::MagnetometerSample> mag_sub(0);
    run_for(300'000);
    EXPECT_TRUE(imu.status().mag_configured);

    vk::msg::MagnetometerSample mag_sample{};
    drain_mag(mag_sub, mag_sample);

    const auto failures_before = imu.status().mag_failures;
    // Inject stray bytes so FIFO count is not a multiple of 9.
    model.push_fifo_byte(0xAA);
    run_for(period_us);

    EXPECT_EQ(imu.status().mag_failures, failures_before + 1);

    // Run for 30 ms so fresh records enter the FIFO and valid samples resume.
    run_for(30'000);
    EXPECT_GT(drain_mag(mag_sub, mag_sample), 0);
    EXPECT_EQ(mag_sample.failures, failures_before + 1);
}

// A magnetometer that stops answering after setup is detected, counted, and retried.
TEST_F(Icm20948Test, MagnetometerThatStopsAfterSetupIsRetried)
{
    Icm20948 imu(vk::hal::I2cDevice(bus, 0x68), {.rate_hz = rate_hz}, queue);
    imu.start();
    vk::core::Subscription<vk::msg::ImuSample> imu_sub(0);
    vk::core::Subscription<vk::msg::MagnetometerSample> mag_sub(0);
    run_for(300'000);
    EXPECT_TRUE(imu.status().mag_configured);

    // Magnetometer stops answering.
    model.set_mag_responding(false);

    // Run for 110 ms (exceeds the 100 ms timeout for no data ready).
    run_for(110'000);
    EXPECT_FALSE(imu.status().mag_configured);
    EXPECT_GT(imu.status().mag_failures, 0U);

    // While unresponsive, IMU keeps publishing samples every run.
    vk::msg::ImuSample imu_sample{};
    drain(imu_sub, imu_sample);
    run_for(200'000);
    EXPECT_GT(drain(imu_sub, imu_sample), 0);
    EXPECT_EQ(imu.status().state, State::Running);

    // Magnetometer responds again and setup recovers.
    model.set_mag_responding(true);
    run_for(250'000);
    EXPECT_TRUE(imu.status().mag_configured);
    vk::msg::MagnetometerSample mag_sample{};
    EXPECT_GT(drain_mag(mag_sub, mag_sample), 0);
    EXPECT_GT(mag_sample.failures, 0U);
}

// With no new samples produced by the AK09916, the driver publishes nothing
// while imu_sample continues; with new samples, it publishes again.
TEST_F(Icm20948Test, MagnetometerSamplePublishedOnlyOnDataReady)
{
    Icm20948 imu(vk::hal::I2cDevice(bus, 0x68), {.rate_hz = rate_hz}, queue);
    imu.start();
    vk::core::Subscription<vk::msg::ImuSample> imu_sub(0);
    vk::core::Subscription<vk::msg::MagnetometerSample> mag_sub(0);
    run_for(300'000);
    EXPECT_TRUE(imu.status().mag_configured);

    vk::msg::ImuSample imu_sample{};
    vk::msg::MagnetometerSample mag_sample{};
    drain(imu_sub, imu_sample);
    drain_mag(mag_sub, mag_sample);

    // Pause magnetometer sampling: IMU continues while mag publishes nothing.
    model.set_mag_paused(true);
    int imu_count = 0;
    int mag_count = 0;
    for (int i = 0; i < 10; ++i) {
        run_for(period_us);
        if (drain(imu_sub, imu_sample) > 0) {
            ++imu_count;
        }
        if (drain_mag(mag_sub, mag_sample) > 0) {
            ++mag_count;
        }
    }
    EXPECT_EQ(imu_count, 10);
    EXPECT_EQ(mag_count, 0);

    // Resume magnetometer sampling: mag samples resume.
    model.set_mag_paused(false);
    imu_count = 0;
    mag_count = 0;
    for (int i = 0; i < 10; ++i) {
        run_for(period_us);
        if (drain(imu_sub, imu_sample) > 0) {
            ++imu_count;
        }
        if (drain_mag(mag_sub, mag_sample) > 0) {
            ++mag_count;
        }
    }
    EXPECT_EQ(imu_count, 10);
    EXPECT_GT(mag_count, 0);
}

// Overflow is counted and dropped.
TEST_F(Icm20948Test, MagnetometerOverflowCountedAndSampleDropped)
{
    Icm20948 imu(vk::hal::I2cDevice(bus, 0x68), {.rate_hz = rate_hz}, queue);
    imu.start();
    vk::core::Subscription<vk::msg::MagnetometerSample> mag_sub(0);
    run_for(300'000);

    vk::msg::MagnetometerSample mag_sample{};
    drain_mag(mag_sub, mag_sample);

    model.set_mag_overflow(true);
    run_for(50'000);
    EXPECT_FALSE(mag_sub.updated());
    EXPECT_GE(imu.status().mag_overflows, 5U);

    model.set_mag_overflow(false);
    run_for(20'000);
    ASSERT_GT(drain_mag(mag_sub, mag_sample), 0);
    EXPECT_GE(mag_sample.overflows, 5U);
}

// Axis signs against true attitude at four headings and upside down.
TEST_F(Icm20948Test, MagnetometerAxisSignsAtFourHeadingsAndUpsideDown)
{
    Icm20948 imu(vk::hal::I2cDevice(bus, 0x68), {.rate_hz = rate_hz}, queue);
    imu.start();
    vk::core::Subscription<vk::msg::MagnetometerSample> sub(0);
    run_for(300'000);

    // Constant Earth field in NED: 30 µT north, 0 east, 30 µT down (spec 0012).
    // The sensor frame is x forward, y left, z up; body frame is x forward, y right, z down.
    // Therefore: sensor_x = body_x, sensor_y = -body_y, sensor_z = -body_z.
    struct TestCase {
        std::string_view name;
        std::array<float, 3> sensor_field;  // in sensor frame (passed to model)
        std::array<float, 3> expected_body; // in body frame (expected in sample)
    };

    const std::array<TestCase, 5> cases{{
        {.name = "Heading 0 deg",
         .sensor_field = {30.0F, 0.0F, -30.0F},
         .expected_body = {30.0F, 0.0F, 30.0F}},
        {.name = "Heading 90 deg",
         .sensor_field = {0.0F, 30.0F, -30.0F},
         .expected_body = {0.0F, -30.0F, 30.0F}},
        {.name = "Heading 180 deg",
         .sensor_field = {-30.0F, 0.0F, -30.0F},
         .expected_body = {-30.0F, 0.0F, 30.0F}},
        {.name = "Heading 270 deg",
         .sensor_field = {0.0F, -30.0F, -30.0F},
         .expected_body = {0.0F, 30.0F, 30.0F}},
        {.name = "Upside down",
         .sensor_field = {30.0F, 0.0F, 30.0F},
         .expected_body = {30.0F, 0.0F, -30.0F}},
    }};

    for (const auto& tc : cases) {
        model.set_field(tc.sensor_field);
        run_for(20'000);  // 20 ms ensures at least one 10 ms mag sample
        vk::msg::MagnetometerSample sample{};
        ASSERT_GT(drain_mag(sub, sample), 0) << tc.name;
        EXPECT_NEAR(sample.field[0], tc.expected_body[0], mag_lsb) << tc.name;
        EXPECT_NEAR(sample.field[1], tc.expected_body[1], mag_lsb) << tc.name;
        EXPECT_NEAR(sample.field[2], tc.expected_body[2], mag_lsb) << tc.name;
    }
}

// Body-frame calibration parameters applied to output samples.
TEST_F(Icm20948Test, MagnetometerCalibrationAppliedInBodyFrame)
{
    vk::params::cal.mag_xoff = 5.0F;
    vk::params::cal.mag_yoff = -3.0F;
    vk::params::cal.mag_zoff = 2.0F;
    vk::params::cal.mag_xscale = 1.1F;
    vk::params::cal.mag_yscale = 0.9F;
    vk::params::cal.mag_zscale = 1.05F;

    // Body field before calibration: {30, 0, 30} -> sensor field {30, 0, -30}.
    model.set_field({30.0F, 0.0F, -30.0F});

    Icm20948 imu(vk::hal::I2cDevice(bus, 0x68), {.rate_hz = rate_hz}, queue);
    imu.start();
    vk::core::Subscription<vk::msg::MagnetometerSample> sub(0);
    run_for(300'000);

    vk::msg::MagnetometerSample sample{};
    ASSERT_GT(drain_mag(sub, sample), 0);
    // (30 - 5) * 1.1 = 27.5
    // (0 - (-3)) * 0.9 = 2.7
    // (30 - 2) * 1.05 = 29.4
    EXPECT_NEAR(sample.field[0], 27.5F, mag_lsb);
    EXPECT_NEAR(sample.field[1], 2.7F, mag_lsb);
    EXPECT_NEAR(sample.field[2], 29.4F, mag_lsb);

    // Changes to parameters apply from the next sample without reboot.
    vk::params::cal.mag_xoff = 10.0F;
    run_for(20'000);
    ASSERT_GT(drain_mag(sub, sample), 0);
    // (30 - 10) * 1.1 = 22.0
    EXPECT_NEAR(sample.field[0], 22.0F, mag_lsb);
}

// A missing AK09916 leaves imu_sample running and is retried.
TEST_F(Icm20948Test, MissingAk09916LeavesImuRunningAndIsRetried)
{
    model.set_mag_responding(false);
    Icm20948 imu(vk::hal::I2cDevice(bus, 0x68), {.rate_hz = rate_hz}, queue);
    imu.start();
    vk::core::Subscription<vk::msg::ImuSample> imu_sub(0);
    vk::core::Subscription<vk::msg::MagnetometerSample> mag_sub(0);

    run_for(300'000);
    EXPECT_EQ(imu.status().state, State::Running);
    vk::msg::ImuSample imu_sample{};
    EXPECT_GT(drain(imu_sub, imu_sample), 0);
    EXPECT_FALSE(mag_sub.updated());
    EXPECT_GT(imu.status().mag_failures, 0U);
    EXPECT_FALSE(imu.status().mag_configured);

    const auto failures_before = imu.status().mag_failures;
    run_for(250'000);
    EXPECT_GT(imu.status().mag_failures, failures_before);
    EXPECT_GT(drain(imu_sub, imu_sample), 0);

    // Magnetometer becomes available.
    model.set_mag_responding(true);
    run_for(250'000);
    EXPECT_TRUE(imu.status().mag_configured);
    vk::msg::MagnetometerSample mag_sample{};
    EXPECT_GT(drain_mag(mag_sub, mag_sample), 0);
    EXPECT_GT(mag_sample.failures, 0U);
}
#endif

}  // namespace
