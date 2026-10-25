// SPDX-License-Identifier: GPL-3.0-or-later
// The commander work item's reboot service on a fake clock (ADR-0023): a
// reboot is refused while armed, happens reboot_delay_us after it is
// accepted so the reply leaves first, and arming is refused meanwhile. And
// the reset cause it publishes on vehicle_status (ADR-0005). And both
// setpoint levels, direct-actuator and body velocity, keeping the setpoint
// timeout and the idle disarm (ADR-0005).
#include <vk/commander/commander_task.hpp>
#include <vk/core/bus.hpp>
#include <vk/core/fault.hpp>
#include <vk/core/vehicle_control.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/hal/reset.hpp>
#include <vk/msg/actuator_setpoint.hpp>
#include <vk/msg/body_velocity_setpoint.hpp>
#include <vk/msg/imu_sample.hpp>
#include <vk/msg/vehicle_status.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

namespace {

using vk::commander::CommanderTask;
using vk::core::ArmingResult;
using vk::core::RebootAction;
using vk::core::RebootResult;
using vk::hal::BootTarget;

constexpr std::uint32_t interval_us = 5'000;
constexpr vk::core::ControlSource link0 = 0;

std::uint64_t fake_now = 0;

std::uint64_t fake_clock() noexcept
{
    return fake_now;
}

std::vector<BootTarget> resets;

void fake_reset(BootTarget target)
{
    resets.push_back(target);  // A real reset does not return.
}

class CommanderTaskTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        fake_now = 1'000'000;
        resets.clear();
        task.start();
        // The bus outlives each test: replace what earlier tests left on the setpoints.
        body.publish(vk::msg::BodyVelocitySetpoint{.timestamp = 0, .vx = NAN, .vy = NAN, .yaw_rate = NAN});
        direct.publish(vk::msg::ActuatorSetpoint{.timestamp = 0, .wheel_speed = {NAN, NAN, NAN, NAN}});
        imu.publish(vk::msg::ImuSample{.timestamp = fake_now});  // the IMU answered: the board verifies
        step(interval_us);
    }

    // Runs the queue every control cycle for `duration_us`.
    void step(std::uint64_t duration_us)
    {
        for (const std::uint64_t end = fake_now + duration_us; fake_now < end;) {
            fake_now += interval_us;
            queue.process();
        }
    }

    void arm()
    {
        ASSERT_EQ(task.request_mode(link0, vk::core::Mode::Test), vk::core::ModeResult::Ok);
        ASSERT_EQ(task.request_arming(link0, true), ArmingResult::Ok);
    }

    vk::core::WorkQueue queue{"wq:test", 1, &fake_clock};
    // Sends a body velocity setpoint every control cycle for `duration_us`, as
    // setpoint arbitration publishes them, stamped when accepted.
    void send_body(float vx, float yaw_rate, std::uint64_t duration_us)
    {
        for (const std::uint64_t end = fake_now + duration_us; fake_now < end;) {
            body.publish(vk::msg::BodyVelocitySetpoint{.timestamp = fake_now, .vx = vx, .vy = 0.0F, .yaw_rate = yaw_rate});
            step(interval_us);
        }
    }

    vk::core::Publication<vk::msg::ImuSample> imu{0};
    vk::core::Publication<vk::msg::BodyVelocitySetpoint> body{0};
    vk::core::Publication<vk::msg::ActuatorSetpoint> direct{0};
    CommanderTask task{{.interval_us = interval_us, .actuators_started = true, .reset = &fake_reset}, queue};
};

TEST_F(CommanderTaskTest, ARebootIsRefusedWhileArmed)
{
    arm();
    EXPECT_EQ(task.request_reboot(RebootAction::Bootloader), RebootResult::Armed);
    step(500'000);
    EXPECT_TRUE(resets.empty());
}

TEST_F(CommanderTaskTest, AnAcceptedRebootHappensAfterTheDelayWithItsTarget)
{
    ASSERT_EQ(task.request_reboot(RebootAction::Bootloader), RebootResult::Ok);
    step(CommanderTask::reboot_delay_us - interval_us);
    EXPECT_TRUE(resets.empty()) << "the reply must leave before the reset";
    step(interval_us);
    ASSERT_FALSE(resets.empty());
    EXPECT_EQ(resets.front(), BootTarget::Bootloader);
}

TEST_F(CommanderTaskTest, APlainRebootStartsTheFirmware)
{
    ASSERT_EQ(task.request_reboot(RebootAction::Reboot), RebootResult::Ok);
    step(CommanderTask::reboot_delay_us);
    ASSERT_FALSE(resets.empty());
    EXPECT_EQ(resets.front(), BootTarget::Firmware);
}

TEST_F(CommanderTaskTest, ArmingIsRefusedWhileARebootIsPending)
{
    ASSERT_EQ(task.request_mode(link0, vk::core::Mode::Test), vk::core::ModeResult::Ok);
    ASSERT_EQ(task.request_reboot(RebootAction::Reboot), RebootResult::Ok);
    EXPECT_EQ(task.request_arming(link0, true), ArmingResult::Rebooting);
    EXPECT_FALSE(task.armed());
}

TEST_F(CommanderTaskTest, ARepeatedRequestKeepsTheFirstDeadlineAndTakesTheNewTarget)
{
    ASSERT_EQ(task.request_reboot(RebootAction::Reboot), RebootResult::Ok);
    step(CommanderTask::reboot_delay_us / 2);
    ASSERT_EQ(task.request_reboot(RebootAction::Bootloader), RebootResult::Ok);
    step(CommanderTask::reboot_delay_us / 2);
    ASSERT_FALSE(resets.empty());
    EXPECT_EQ(resets.front(), BootTarget::Bootloader);
}

TEST(CommanderTaskWithoutReset, ARebootIsNotAvailable)
{
    fake_now = 1'000'000;
    vk::core::WorkQueue queue{"wq:test", 1, &fake_clock};
    CommanderTask task{{.interval_us = interval_us}, queue};
    EXPECT_EQ(task.request_reboot(RebootAction::Reboot), RebootResult::NotAvailable);
}

TEST(CommanderTaskAfterAFault, PublishesTheResetCauseAndRefusesArming)
{
    fake_now = 1'000'000;
    vk::core::WorkQueue queue{"wq:test", 1, &fake_clock};
    vk::core::Publication<vk::msg::ImuSample> imu{0};
    vk::core::Subscription<vk::msg::VehicleStatus> status{0};
    CommanderTask task{{.interval_us = interval_us,
                        .actuators_started = true,
                        .reset_report = {.cause = vk::core::ResetCause::Fault,
                                         .fault = vk::core::Fault::Assert,
                                         .fault_address = 0x0801ABCD}},
                       queue};
    task.start();
    imu.publish(vk::msg::ImuSample{.timestamp = fake_now});
    fake_now += interval_us;
    queue.process();
    vk::msg::VehicleStatus published{};
    ASSERT_TRUE(status.copy(published));
    EXPECT_EQ(published.reset_cause, static_cast<std::uint8_t>(vk::core::ResetCause::Fault));
    EXPECT_EQ(published.fault, static_cast<std::uint8_t>(vk::core::Fault::Assert));
    EXPECT_EQ(published.fault_address, 0x0801ABCDU);
    ASSERT_EQ(task.request_mode(link0, vk::core::Mode::Test), vk::core::ModeResult::Ok);
    EXPECT_EQ(task.request_arming(link0, true), ArmingResult::UnexpectedReset);
}

}  // namespace

TEST_F(CommanderTaskTest, BodyVelocitySetpointsKeepTheVehicleArmedAndTimeOut)
{
    arm();
    send_body(0.5F, 0.0F, 1'000'000);
    EXPECT_TRUE(task.armed());
    // They stop: after COM_SP_TIMEOUT (0.5 s) the vehicle disarms for it.
    step(400'000);
    EXPECT_TRUE(task.armed());
    step(200'000);
    EXPECT_FALSE(task.armed());
    vk::msg::VehicleStatus status{};
    vk::core::Subscription<vk::msg::VehicleStatus> statuses{0};
    ASSERT_TRUE(statuses.copy(status));
    EXPECT_EQ(status.disarm_reason, static_cast<std::uint8_t>(vk::core::DisarmReason::SetpointTimeout));
}

TEST_F(CommanderTaskTest, TheLevelAcceptedLastDecidesWhetherTheTargetsAreIdle)
{
    arm();
    // A zero body velocity after a moving direct-actuator setpoint is idle:
    // the vehicle disarms after COM_DISARM_IDLE (10 s) of it.
    direct.publish(vk::msg::ActuatorSetpoint{.timestamp = fake_now, .wheel_speed = {5.0F, 5.0F, NAN, NAN}});
    step(interval_us);
    send_body(0.0F, 0.0F, 9'000'000);
    EXPECT_TRUE(task.armed());
    send_body(0.0F, 0.0F, 1'100'000);
    EXPECT_FALSE(task.armed());
}

TEST_F(CommanderTaskTest, ATurnInPlaceIsNotIdle)
{
    arm();
    send_body(0.0F, 1.0F, 10'500'000);
    EXPECT_TRUE(task.armed());
}
