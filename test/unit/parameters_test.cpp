// SPDX-License-Identifier: GPL-3.0-or-later
// The parameter runtime (ADR-0012): lookup, validation, the safety
// attributes and parameter_update, over a table built here; and the table
// the parameter generator emits for this build.
#include <vk/core/bus.hpp>
#include <vk/core/parameters.hpp>
#include <vk/msg/arming_state.hpp>
#include <vk/msg/parameter_update.hpp>
#include <vk/params/params.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <string_view>
#include <vector>

namespace {

using vk::core::ParamInfo;
using vk::core::ParamResult;
using vk::core::Parameters;
using vk::core::ParamType;
using vk::core::ParamValue;
using vk::core::param_hash;
namespace param_flags = vk::core::param_flags;

static_assert(param_hash("") == 0x811C9DC5U);  // FNV-1a 32 reference values
static_assert(param_hash("a") == 0xE40C292CU);
static_assert(param_hash("foobar") == 0xBF9CF968U);

std::uint64_t fake_now = 0;

std::uint64_t fake_clock() noexcept
{
    return fake_now;
}

// Values of the test table, reset before each test.
struct Values {
    std::int32_t count;
    float gain;
    std::int32_t lock;
    std::int32_t mask;
    std::int32_t mode;
};
Values values{};
constexpr Values defaults{.count = 3, .gain = 0.5F, .lock = 1, .mask = 5, .mode = 0};

constexpr std::array<std::int32_t, 2> lock_values{0, 1};
constexpr std::array<std::int32_t, 3> mode_values{0, 1, -1};

ParamInfo int_param(std::string_view name, std::int32_t* value, std::int32_t min, std::int32_t max)
{
    return {.name = name,
            .hash = param_hash(name),
            .type = ParamType::Int32,
            .flags = 0,
            .default_value = ParamValue::from_int(*value),
            .min = ParamValue::from_int(min),
            .max = ParamValue::from_int(max),
            .enum_values = {},
            .bitmask = 0,
            .int_value = value,
            .float_value = nullptr};
}

// Sorted by name, as the generator emits it.
std::array<ParamInfo, 5> make_table()
{
    ParamInfo count = int_param("T_COUNT", &values.count, -10, 10);
    count.flags = param_flags::reboot_required;
    ParamInfo gain{.name = "T_GAIN",
                   .hash = param_hash("T_GAIN"),
                   .type = ParamType::Float,
                   .flags = 0,
                   .default_value = ParamValue::from_float(defaults.gain),
                   .min = ParamValue::from_float(0.0F),
                   .max = ParamValue::from_float(2.0F),
                   .enum_values = {},
                   .bitmask = 0,
                   .int_value = nullptr,
                   .float_value = &values.gain};
    ParamInfo lock = int_param("T_LOCK", &values.lock, 0, 1);
    lock.flags = param_flags::locked_when_armed;
    lock.enum_values = lock_values;
    ParamInfo mask = int_param("T_MASK", &values.mask, 0, 5);
    mask.bitmask = 0x5;
    ParamInfo mode = int_param("T_MODE", &values.mode, -1, 1);
    mode.enum_values = mode_values;
    return {count, gain, lock, mask, mode};
}

class ParametersTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        values = defaults;
        fake_now = 1'000;
        disarm();
        while (updates.copy(last_update)) {
        }
    }

    void TearDown() override { disarm(); }

    static void arm(bool armed)
    {
        vk::msg::ArmingState state{};
        state.timestamp = fake_now;
        state.armed = armed;
        vk::core::Publication<vk::msg::ArmingState>{}.publish(state);
    }
    static void disarm() { arm(false); }

    // parameter_update publications since the last call.
    int new_updates()
    {
        const std::uint64_t generation = vk::core::topic<vk::msg::ParameterUpdate>(0).generation();
        const auto count = static_cast<int>(generation - seen_);
        seen_ = generation;
        while (updates.copy(last_update)) {
        }
        return count;
    }

    std::array<ParamInfo, 5> table = make_table();
    Parameters parameters{table, &fake_clock};
    vk::core::Subscription<vk::msg::ParameterUpdate> updates;
    vk::msg::ParameterUpdate last_update{};
    std::uint64_t seen_ = vk::core::topic<vk::msg::ParameterUpdate>(0).generation();
};

TEST_F(ParametersTest, FindsParametersByName)
{
    EXPECT_EQ(parameters.count(), 5U);
    EXPECT_EQ(parameters.find("T_COUNT"), 0U);
    EXPECT_EQ(parameters.find("T_MODE"), 4U);
    EXPECT_FALSE(parameters.find("T_NONE").has_value());
    EXPECT_FALSE(parameters.find("T_").has_value());
    EXPECT_EQ(parameters.set("T_NONE", ParamValue::from_int(1)), ParamResult::UnknownName);
}

TEST_F(ParametersTest, GetsTheCurrentValues)
{
    EXPECT_EQ(parameters.get(0), ParamValue::from_int(3));
    EXPECT_EQ(parameters.get(1), ParamValue::from_float(0.5F));
    values.count = 7;
    EXPECT_EQ(parameters.get(0).as_int(), 7);
}

TEST_F(ParametersTest, SetWritesTheMemberAndPublishesOneUpdate)
{
    fake_now = 42'000;
    EXPECT_EQ(parameters.set("T_GAIN", ParamValue::from_float(1.25F)), ParamResult::Ok);
    EXPECT_EQ(values.gain, 1.25F);
    EXPECT_EQ(new_updates(), 1);
    EXPECT_EQ(last_update.timestamp, 42'000U);
    EXPECT_EQ(last_update.name_hash, param_hash("T_GAIN"));
    const std::uint32_t first = last_update.change_count;

    EXPECT_EQ(parameters.set(0, ParamValue::from_int(-10)), ParamResult::Ok);
    EXPECT_EQ(values.count, -10);
    EXPECT_EQ(new_updates(), 1);
    EXPECT_EQ(last_update.name_hash, param_hash("T_COUNT"));
    EXPECT_EQ(last_update.change_count, first + 1);
}

TEST_F(ParametersTest, SettingTheCurrentValueAgainPublishesNothing)
{
    EXPECT_EQ(parameters.set("T_COUNT", ParamValue::from_int(3)), ParamResult::Ok);
    EXPECT_EQ(new_updates(), 0);
}

TEST_F(ParametersTest, RejectsAWrongType)
{
    EXPECT_EQ(parameters.set("T_GAIN", ParamValue::from_int(1)), ParamResult::WrongType);
    EXPECT_EQ(parameters.set("T_COUNT", ParamValue::from_float(1.0F)), ParamResult::WrongType);
    EXPECT_EQ(values.gain, defaults.gain);
    EXPECT_EQ(values.count, defaults.count);
    EXPECT_EQ(new_updates(), 0);
}

TEST_F(ParametersTest, RejectsValuesOutOfRange)
{
    EXPECT_EQ(parameters.set("T_COUNT", ParamValue::from_int(11)), ParamResult::OutOfRange);
    EXPECT_EQ(parameters.set("T_COUNT", ParamValue::from_int(-11)), ParamResult::OutOfRange);
    EXPECT_EQ(parameters.set("T_GAIN", ParamValue::from_float(2.5F)), ParamResult::OutOfRange);
    EXPECT_EQ(parameters.set("T_GAIN", ParamValue::from_float(-0.1F)), ParamResult::OutOfRange);
    EXPECT_EQ(parameters.set("T_GAIN", ParamValue::from_float(std::numeric_limits<float>::quiet_NaN())),
              ParamResult::OutOfRange);
    EXPECT_EQ(parameters.set("T_GAIN", ParamValue::from_float(std::numeric_limits<float>::infinity())),
              ParamResult::OutOfRange);
    EXPECT_EQ(values.count, defaults.count);
    EXPECT_EQ(values.gain, defaults.gain);
    EXPECT_EQ(new_updates(), 0);
}

TEST_F(ParametersTest, AcceptsOnlyEnumValuesAndListedBits)
{
    EXPECT_EQ(parameters.set("T_MODE", ParamValue::from_int(-1)), ParamResult::Ok);
    EXPECT_EQ(parameters.set("T_MODE", ParamValue::from_int(2)), ParamResult::OutOfRange);
    EXPECT_EQ(parameters.set("T_MASK", ParamValue::from_int(4)), ParamResult::Ok);
    EXPECT_EQ(parameters.set("T_MASK", ParamValue::from_int(2)), ParamResult::OutOfRange);
    EXPECT_EQ(values.mode, -1);
    EXPECT_EQ(values.mask, 4);
}

TEST_F(ParametersTest, LockedParametersRejectChangesWhileArmed)
{
    arm(true);
    EXPECT_TRUE(parameters.armed());
    EXPECT_EQ(parameters.set("T_LOCK", ParamValue::from_int(0)), ParamResult::LockedWhileArmed);
    EXPECT_EQ(values.lock, 1);
    // Parameters without the attribute still change.
    EXPECT_EQ(parameters.set("T_GAIN", ParamValue::from_float(1.0F)), ParamResult::Ok);

    disarm();
    EXPECT_FALSE(parameters.armed());
    EXPECT_EQ(parameters.set("T_LOCK", ParamValue::from_int(0)), ParamResult::Ok);
    EXPECT_EQ(values.lock, 0);
}

TEST(GeneratedParameters, TableIsSortedAndHashed)
{
    const auto table = vk::params::table();
    ASSERT_EQ(table.size(), vk::params::count);
    for (std::size_t i = 0; i < table.size(); ++i) {
        EXPECT_EQ(table[i].hash, param_hash(table[i].name)) << table[i].name;
        if (i > 0) {
            EXPECT_LT(table[i - 1].name, table[i].name);
        }
    }
    EXPECT_NE(vk::params::set_hash, 0U);
}

TEST(GeneratedParameters, AttitudeGainsStartAtTheirDefaults)
{
    auto& parameters = vk::params::parameters();
    const std::size_t kp = parameters.find("ATT_KP").value_or(parameters.count());
    ASSERT_LT(kp, parameters.count());
    EXPECT_EQ(parameters.info(kp).type, ParamType::Float);
    EXPECT_EQ(parameters.get(kp), parameters.info(kp).default_value);
    EXPECT_EQ(vk::params::att.kp, 1.0F);
    EXPECT_EQ(vk::params::att.ki, 0.05F);
    EXPECT_EQ(vk::params::att.acc_rej, 0.1F);
}

// The local menu's parameters (ADR-0022): the whitelist this build compiles
// in, in table order, with steps for numbers and value names for enums.
TEST(GeneratedParameters, MenuListsTheWhitelistedParameters)
{
    const auto table = vk::params::table();
    std::vector<std::string_view> names;
    std::size_t previous = 0;
    for (const vk::core::MenuParam& entry : vk::params::menu()) {
        ASSERT_LT(entry.index, table.size());
        if (!names.empty()) {
            EXPECT_GT(entry.index, previous);
        }
        previous = entry.index;
        const ParamInfo& info = table[entry.index];
        names.push_back(info.name);
        if (info.enum_values.empty()) {
            EXPECT_EQ(entry.step.type, info.type) << info.name;
            EXPECT_TRUE(entry.value_names.empty()) << info.name;
        } else {
            EXPECT_EQ(entry.value_names.size(), info.enum_values.size()) << info.name;
        }
    }
    // SERIAL1_BAUD is a menu parameter too, but only the STM32F4 platform compiles it in.
    EXPECT_EQ(names, (std::vector<std::string_view>{"ATT_ACC_REJ", "ATT_KI", "ATT_KP", "SENS_BOARD_ROT"}));

    const auto& rot = vk::params::menu().back();
    const ParamInfo& info = table[rot.index];
    ASSERT_EQ(info.enum_values.size(), rot.value_names.size());
    const auto yaw90 = std::ranges::find(rot.value_names, "YAW_90");
    ASSERT_NE(yaw90, rot.value_names.end());
    EXPECT_EQ(info.enum_values[static_cast<std::size_t>(yaw90 - rot.value_names.begin())], 2);
    EXPECT_FLOAT_EQ(vk::params::menu().front().step.as_float(), 0.01F);
}

TEST(GeneratedParameters, NoActuatorOrCommanderParameterIsInTheMenu)
{
    const auto table = vk::params::table();
    for (const vk::core::MenuParam& entry : vk::params::menu()) {
        const std::string_view name = table[entry.index].name;
        EXPECT_FALSE(name.starts_with("MOT")) << name;
        EXPECT_FALSE(name.starts_with("COM")) << name;
    }
}

}  // namespace
