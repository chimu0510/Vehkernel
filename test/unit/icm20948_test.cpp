// SPDX-License-Identifier: GPL-3.0-or-later
// ICM20948 driver against the device's register model on the simulated
// peripheral bus (ADR-0018 L1). The driver's work queue runs on a fake clock.
#include <vk/core/bus.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/drivers/icm20948.hpp>
#include <vk/hal/i2c.hpp>
#include <vk/msg/imu_sample.hpp>
#include <vk/sim/i2c_bus.hpp>
#include <vk/sim/icm20948.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <numbers>

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

class Icm20948Test : public ::testing::Test {
protected:
    void SetUp() override
    {
        fake_now = 1'000'000;
        bus.attach(vk::sim::Icm20948::default_address, model);
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

    vk::sim::I2cBus bus;
    vk::sim::Icm20948 model;
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

}  // namespace
