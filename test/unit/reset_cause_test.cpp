// SPDX-License-Identifier: GPL-3.0-or-later
// Unit tests for the STM32F4 reset cause (ADR-0005): RCC_CSR's reset flags
// and the fault stop's record decoded into the reset report.
#include <vk/platform/reset_cause.hpp>

#include <gtest/gtest.h>

#include <cstdint>

namespace {

using vk::core::Fault;
using vk::core::ResetCause;
using vk::core::ResetReport;
using vk::platform::decode_reset;
using vk::platform::fault_record;
namespace flags = vk::platform::reset_flags;

constexpr std::uint32_t address = 0x08012345;

TEST(ResetCause, EachFlagIsItsCauseAndThePinFlagCountsOnlyAlone)
{
    EXPECT_EQ(decode_reset(flags::power_on | flags::brownout | flags::pin, 0, 0).cause, ResetCause::PowerOn);
    EXPECT_EQ(decode_reset(flags::brownout | flags::pin, 0, 0).cause, ResetCause::PowerOn);
    EXPECT_EQ(decode_reset(flags::pin, 0, 0).cause, ResetCause::Pin);
    EXPECT_EQ(decode_reset(flags::software | flags::pin, 0, 0).cause, ResetCause::Software);
    EXPECT_EQ(decode_reset(flags::iwdg | flags::pin, 0, 0).cause, ResetCause::Watchdog);
    EXPECT_EQ(decode_reset(flags::wwdg | flags::pin, 0, 0).cause, ResetCause::Watchdog);
    EXPECT_EQ(decode_reset(flags::low_power | flags::pin, 0, 0).cause, ResetCause::LowPower);
    EXPECT_EQ(decode_reset(0, 0, 0), ResetReport{});
}

TEST(ResetCause, AWatchdogBeatsASoftwareResetFlaggedWithIt)
{
    EXPECT_EQ(decode_reset(flags::iwdg | flags::software | flags::pin, 0, 0).cause, ResetCause::Watchdog);
}

TEST(ResetCause, AWatchdogResetAfterTheFaultStopIsAFaultWithItsRecord)
{
    for (const Fault fault : {Fault::HardFault, Fault::MemManage, Fault::BusFault, Fault::UsageFault, Fault::Assert,
                              Fault::Fail, Fault::StackOverflow, Fault::UnexpectedInterrupt}) {
        const ResetReport report = decode_reset(flags::iwdg | flags::pin, fault_record(fault), address);
        EXPECT_EQ(report, (ResetReport{.cause = ResetCause::Fault, .fault = fault, .fault_address = address}))
            << static_cast<int>(fault);
        EXPECT_TRUE(vk::core::unexpected(report));
    }
    EXPECT_EQ(decode_reset(flags::software | flags::pin, fault_record(Fault::Assert), address).cause,
              ResetCause::Fault);
}

TEST(ResetCause, ARecordOutsideAWatchdogOrSoftwareResetIsStale)
{
    const std::uint32_t record = fault_record(Fault::HardFault);
    EXPECT_EQ(decode_reset(flags::power_on | flags::brownout | flags::pin, record, address),
              ResetReport{.cause = ResetCause::PowerOn});
    EXPECT_EQ(decode_reset(flags::pin, record, address), ResetReport{.cause = ResetCause::Pin});
}

TEST(ResetCause, ABrokenRecordDoesNotMakeAFault)
{
    EXPECT_EQ(decode_reset(flags::iwdg, 0xDEADBEEF, address), ResetReport{.cause = ResetCause::Watchdog});
    EXPECT_EQ(decode_reset(flags::iwdg, fault_record(Fault::None), address), ResetReport{.cause = ResetCause::Watchdog});
    EXPECT_EQ(decode_reset(flags::iwdg, vk::platform::fault_record_marker | 0xFFU, address),
              ResetReport{.cause = ResetCause::Watchdog});
}

TEST(ResetCause, OnlyWatchdogAndFaultResetsAreUnexpected)
{
    EXPECT_TRUE(vk::core::unexpected({.cause = ResetCause::Watchdog}));
    EXPECT_TRUE(vk::core::unexpected({.cause = ResetCause::Fault}));
    for (const ResetCause cause :
         {ResetCause::Unknown, ResetCause::PowerOn, ResetCause::Pin, ResetCause::Software, ResetCause::LowPower}) {
        EXPECT_FALSE(vk::core::unexpected({.cause = cause})) << static_cast<int>(cause);
    }
}

}  // namespace
