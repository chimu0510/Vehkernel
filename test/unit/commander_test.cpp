// Unit tests for the commander's state machine (ADR-0005, spec 0006): modes,
// arming checks, actuator power sense, the emergency stop and the timers.
#include <vk/commander/commander.hpp>

#include <gtest/gtest.h>

#include <cstdint>

namespace {

using vk::commander::ActuatorPower;
using vk::commander::Commander;
using vk::commander::Inputs;
using vk::commander::Settings;
using vk::core::ArmingResult;
using vk::core::DisarmReason;
using vk::core::Mode;
using vk::core::ModeResult;

constexpr std::uint8_t link0 = 0;
constexpr std::uint8_t link1 = 1;
constexpr std::uint64_t cycle_us = 5'000;

// A commander driven cycle by cycle, with inputs where every check passes.
class CommanderTest : public ::testing::Test {
protected:
    void step(std::uint64_t duration_us = cycle_us)
    {
        const std::uint64_t end = now + duration_us;
        while (now < end) {
            now += cycle_us;
            commander.update(now, inputs, settings);
        }
    }

    // Powered long enough, in TEST with link0 in control.
    void ready()
    {
        step(Commander::power_settle_us + cycle_us);
        ASSERT_EQ(commander.request_mode(now, link0, Mode::Test), ModeResult::Ok);
    }

    void arm()
    {
        ready();
        ASSERT_EQ(commander.request_arming(now, link0, true), ArmingResult::Ok);
        ASSERT_TRUE(commander.state().armed);
    }

    Commander commander;
    Inputs inputs{.board_verified = true, .actuator_power = true, .actuators_started = true};
    Settings settings;
    std::uint64_t now = 0;
};

TEST_F(CommanderTest, BootsInStandbyWithNoSourceAndRefusesToArm)
{
    step(Commander::power_settle_us + cycle_us);
    EXPECT_EQ(commander.state().mode, Mode::Standby);
    EXPECT_EQ(commander.state().source, vk::core::no_source);
    EXPECT_EQ(commander.request_arming(now, link0, true), ArmingResult::WrongMode);
    EXPECT_FALSE(commander.state().armed);
}

TEST_F(CommanderTest, ArmsInTestFromTheSourceInControl)
{
    arm();
    EXPECT_EQ(commander.state().source, link0);
    EXPECT_EQ(commander.state().arming_refusal, ArmingResult::Ok);
}

TEST_F(CommanderTest, RefusesEachArmingCheckWithItsReasonInOrder)
{
    ready();
    inputs.board_verified = false;
    step();
    EXPECT_EQ(commander.request_arming(now, link0, true), ArmingResult::BoardNotVerified);
    EXPECT_EQ(commander.state().arming_refusal, ArmingResult::BoardNotVerified);
    inputs.board_verified = true;
    step();

    commander.request_emergency_stop(now, true);
    EXPECT_EQ(commander.request_arming(now, link0, true), ArmingResult::EmergencyStop);
    commander.request_emergency_stop(now, false);

    EXPECT_EQ(commander.request_arming(now, link1, true), ArmingResult::NotInControl);

    inputs.setpoints_idle = false;
    inputs.last_setpoint_us = now;
    step();
    EXPECT_EQ(commander.request_arming(now, link0, true), ArmingResult::SetpointsActive);
    // A setpoint older than the setpoint timeout no longer counts.
    step(static_cast<std::uint64_t>(settings.setpoint_timeout_s * 1e6F) + cycle_us);
    EXPECT_NE(commander.request_arming(now, link0, true), ArmingResult::SetpointsActive);
    commander.request_arming(now, link0, false);
    inputs.setpoints_idle = true;

    inputs.bringup_running = true;
    step();
    EXPECT_EQ(commander.request_arming(now, link0, true), ArmingResult::BringupRunning);
    inputs.bringup_running = false;

    inputs.actuator_power = false;
    step();
    EXPECT_EQ(commander.request_arming(now, link0, true), ArmingResult::ActuatorPowerOff);
    inputs.actuator_power = true;
    step(Commander::power_settle_us + cycle_us);

    inputs.actuators_started = false;
    step();
    EXPECT_EQ(commander.request_arming(now, link0, true), ArmingResult::ActuatorsNotStarted);
    EXPECT_FALSE(commander.state().armed);
}

TEST_F(CommanderTest, ModeChangesOnlyWhileDisarmed)
{
    arm();
    EXPECT_EQ(commander.request_mode(now, link0, Mode::Standby), ModeResult::Armed);
    EXPECT_EQ(commander.request_mode(now, link1, Mode::Test), ModeResult::Armed);
    EXPECT_EQ(commander.request_mode(now, link0, Mode::Test), ModeResult::Ok);  // a repeat changes nothing
    EXPECT_EQ(commander.state().mode, Mode::Test);
    EXPECT_EQ(commander.state().source, link0);
}

TEST_F(CommanderTest, StandbyReleasesControl)
{
    ready();
    EXPECT_EQ(commander.request_mode(now, link0, Mode::Standby), ModeResult::Ok);
    EXPECT_EQ(commander.state().source, vk::core::no_source);
}

TEST_F(CommanderTest, DisarmIsAcceptedFromAnySource)
{
    arm();
    EXPECT_EQ(commander.request_arming(now, link1, false), ArmingResult::Ok);
    EXPECT_FALSE(commander.state().armed);
    EXPECT_EQ(commander.state().disarm_reason, DisarmReason::Request);
}

TEST_F(CommanderTest, ActuatorPowerSettlesBeforeItCounts)
{
    inputs.actuator_power = false;
    step();
    EXPECT_EQ(commander.state().actuator_power, ActuatorPower::Unpowered);
    ASSERT_EQ(commander.request_mode(now, link0, Mode::Test), ModeResult::Ok);
    // Settling counts from the first powered sample, one cycle in.
    inputs.actuator_power = true;
    step(Commander::power_settle_us);
    EXPECT_EQ(commander.state().actuator_power, ActuatorPower::Settling);
    EXPECT_EQ(commander.request_arming(now, link0, true), ArmingResult::ActuatorPowerOff);
    step(cycle_us);
    EXPECT_EQ(commander.state().actuator_power, ActuatorPower::Powered);
    EXPECT_EQ(commander.request_arming(now, link0, true), ArmingResult::Ok);
}

TEST_F(CommanderTest, AOneCycleGlitchRestartsTheSettleTime)
{
    inputs.actuator_power = false;
    step();
    inputs.actuator_power = true;
    step(Commander::power_settle_us - 2 * cycle_us);
    inputs.actuator_power = false;
    step();
    inputs.actuator_power = true;
    step(Commander::power_settle_us);
    EXPECT_EQ(commander.state().actuator_power, ActuatorPower::Settling);
    step(cycle_us);
    EXPECT_EQ(commander.state().actuator_power, ActuatorPower::Powered);
}

TEST_F(CommanderTest, LosingActuatorPowerDisarmsInTheCycleItIsSampledAndNeverReArms)
{
    arm();
    inputs.actuator_power = false;
    step();
    EXPECT_FALSE(commander.state().armed);
    EXPECT_EQ(commander.state().disarm_reason, DisarmReason::ActuatorPowerLost);
    inputs.actuator_power = true;
    step(Commander::power_settle_us * 2);
    EXPECT_FALSE(commander.state().armed);
}

TEST_F(CommanderTest, ASkippedPowerCheckNeitherRefusesNorDisarmsButIsReported)
{
    settings.arm_check = 0;
    inputs.actuator_power = false;
    step();
    EXPECT_EQ(commander.state().skipped_checks, vk::core::arm_checks::actuator_power);
    EXPECT_EQ(commander.state().actuator_power, ActuatorPower::Unpowered);
    ASSERT_EQ(commander.request_mode(now, link0, Mode::Test), ModeResult::Ok);
    EXPECT_EQ(commander.request_arming(now, link0, true), ArmingResult::Ok);
    step(cycle_us * 10);
    EXPECT_TRUE(commander.state().armed);
}

TEST_F(CommanderTest, ABoardWithoutPowerSensePassesAsNotAvailable)
{
    inputs.actuator_power.reset();
    step();
    EXPECT_EQ(commander.state().actuator_power, ActuatorPower::NotAvailable);
    ASSERT_EQ(commander.request_mode(now, link0, Mode::Test), ModeResult::Ok);
    EXPECT_EQ(commander.request_arming(now, link0, true), ArmingResult::Ok);
}

TEST_F(CommanderTest, TheOtherChecksStayInForceWithEveryCheckSkipped)
{
    settings.arm_check = 0;
    inputs.board_verified = false;
    step();
    EXPECT_EQ(commander.request_arming(now, link0, true), ArmingResult::BoardNotVerified);
    inputs.board_verified = true;
    step();
    EXPECT_EQ(commander.request_arming(now, link0, true), ArmingResult::WrongMode);
}

TEST_F(CommanderTest, EmergencyStopKeepsTheVehicleArmedForFiveSecondsThenDisarms)
{
    arm();
    inputs.setpoints_idle = false;  // so the idle disarm stays out of the way
    commander.request_emergency_stop(now, true);
    step(Commander::emergency_stop_disarm_us - cycle_us);
    EXPECT_TRUE(commander.state().armed);
    EXPECT_TRUE(commander.state().emergency_stop);
    step(cycle_us);
    EXPECT_FALSE(commander.state().armed);
    EXPECT_EQ(commander.state().disarm_reason, DisarmReason::EmergencyStop);
    // It stays engaged across disarming until released.
    EXPECT_TRUE(commander.state().emergency_stop);
    inputs.setpoints_idle = true;
    step();
    EXPECT_EQ(commander.request_arming(now, link0, true), ArmingResult::EmergencyStop);
    EXPECT_FALSE(commander.request_emergency_stop(now, false));
    EXPECT_EQ(commander.request_arming(now, link0, true), ArmingResult::Ok);
}

TEST_F(CommanderTest, EmergencyStopReleasedWithinFiveSecondsKeepsTheVehicleArmed)
{
    arm();
    inputs.setpoints_idle = false;
    commander.request_emergency_stop(now, true);
    step(Commander::emergency_stop_disarm_us / 2);
    commander.request_emergency_stop(now, false);
    step(Commander::emergency_stop_disarm_us);
    EXPECT_TRUE(commander.state().armed);
}

TEST_F(CommanderTest, IdleDisarmsAfterDisarmIdleWithNoSetpoint)
{
    arm();
    const auto idle_us = static_cast<std::uint64_t>(settings.disarm_idle_s * 1e6F);
    step(idle_us - cycle_us);
    EXPECT_TRUE(commander.state().armed);
    step(cycle_us);
    EXPECT_FALSE(commander.state().armed);
    EXPECT_EQ(commander.state().disarm_reason, DisarmReason::Idle);
}

TEST_F(CommanderTest, SetpointTimeoutCountsFromTheLastSetpoint)
{
    arm();
    inputs.setpoints_idle = false;
    inputs.last_setpoint_us = now;
    const auto timeout_us = static_cast<std::uint64_t>(settings.setpoint_timeout_s * 1e6F);
    step(timeout_us);
    EXPECT_TRUE(commander.state().armed);
    step(cycle_us);
    EXPECT_FALSE(commander.state().armed);
    EXPECT_EQ(commander.state().disarm_reason, DisarmReason::SetpointTimeout);
}

TEST_F(CommanderTest, TheEmergencyStopHoldsTheSetpointTimeout)
{
    arm();
    inputs.setpoints_idle = false;
    inputs.last_setpoint_us = now;
    commander.request_emergency_stop(now, true);
    step(Commander::emergency_stop_disarm_us / 2);
    commander.request_emergency_stop(now, false);
    step(cycle_us);
    EXPECT_TRUE(commander.state().armed);
}

using vk::core::BringupStart;

constexpr std::uint8_t motors_case = 1;

TEST_F(CommanderTest, ABringupCaseStartsOnlyFromTheArmedSourceInControl)
{
    inputs.bringup_available = true;
    step();
    EXPECT_EQ(commander.request_bringup(link0, motors_case), BringupStart::WrongMode);
    arm();
    EXPECT_EQ(commander.request_bringup(link0, 9), BringupStart::UnknownCase);
    EXPECT_EQ(commander.request_bringup(link1, motors_case), BringupStart::NotInControl);
    EXPECT_EQ(commander.request_bringup(link0, motors_case), BringupStart::Ok);
    commander.request_arming(now, link0, false);
    EXPECT_EQ(commander.request_bringup(link0, motors_case), BringupStart::NotArmed);
}

TEST_F(CommanderTest, ABringupCaseNeedsToBeBuiltInAndNotRunning)
{
    arm();
    EXPECT_EQ(commander.request_bringup(link0, motors_case), BringupStart::NotAvailable);
    inputs.bringup_available = true;
    inputs.bringup_running = true;
    step();
    EXPECT_EQ(commander.request_bringup(link0, motors_case), BringupStart::Running);
    // While it runs: no setpoints, no idle or setpoint timeout, no arming.
    EXPECT_FALSE(commander.accepts_setpoint(link0));
    step(static_cast<std::uint64_t>(settings.disarm_idle_s * 1e6F) + cycle_us);
    EXPECT_TRUE(commander.state().armed);
}

TEST_F(CommanderTest, TheEmergencyStopDuringABringupCaseDisarmsAtOnce)
{
    arm();
    inputs.bringup_running = true;
    step();
    commander.request_emergency_stop(now, true);
    step();
    EXPECT_FALSE(commander.state().armed);
    EXPECT_EQ(commander.state().disarm_reason, DisarmReason::BringupAbort);
}

TEST_F(CommanderTest, ItsSourceFallingSilentAbortsABringupCase)
{
    arm();
    commander.source_lost(link0);
    EXPECT_TRUE(commander.state().armed);  // no case running: nothing to abort
    inputs.bringup_running = true;
    step();
    commander.source_lost(link1);
    EXPECT_TRUE(commander.state().armed);
    commander.source_lost(link0);
    EXPECT_FALSE(commander.state().armed);
    EXPECT_EQ(commander.state().disarm_reason, DisarmReason::BringupAbort);
}

}  // namespace
