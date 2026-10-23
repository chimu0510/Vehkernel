// SPDX-License-Identifier: GPL-3.0-or-later
// The user key's presses and the local menu (ADR-0022, spec 0007): debounce,
// short and long presses and repeats; opening, stepping, saving through the
// parameter service, the idle close and closing on arming; and what the menu
// shows.
#include <vk/core/bus.hpp>
#include <vk/core/parameters.hpp>
#include <vk/local_display/local_menu.hpp>
#include <vk/local_display/pages.hpp>
#include <vk/local_display/user_key.hpp>
#include <vk/msg/arming_state.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace {

using vk::core::MenuParam;
using vk::core::ParamInfo;
using vk::core::ParamResult;
using vk::core::ParamType;
using vk::core::ParamValue;
using vk::local_display::KeyEvent;
using vk::local_display::LocalMenu;
using vk::local_display::MenuAction;
using vk::local_display::MenuState;
using vk::local_display::Page;
using vk::local_display::TextScreen;
using vk::local_display::UserKey;

constexpr std::uint64_t tick = UserKey::sample_interval_us;

// Samples the key every 10 ms for `duration_us` at `pressed`, collecting events.
std::vector<KeyEvent> hold(UserKey& key, std::uint64_t& now, std::uint64_t duration_us, bool pressed)
{
    std::vector<KeyEvent> events;
    for (std::uint64_t end = now + duration_us; now < end; now += tick) {
        const KeyEvent event = key.sample(now, pressed);
        if (event != KeyEvent::None) {
            events.push_back(event);
        }
    }
    return events;
}

class UserKeyTest : public ::testing::Test {
protected:
    void SetUp() override { hold(key, now, 100'000, false); }

    UserKey key;
    std::uint64_t now = 0;
};

TEST_F(UserKeyTest, ALevelCountsAfterThreeSamplesInARow)
{
    EXPECT_TRUE(hold(key, now, 2 * tick, true).empty());
    EXPECT_TRUE(hold(key, now, 100'000, false).empty());
    EXPECT_TRUE(hold(key, now, 3 * tick, true).empty());
    EXPECT_EQ(hold(key, now, 100'000, false), std::vector{KeyEvent::Short});
}

TEST_F(UserKeyTest, ShortBeforeOneSecondLongAtOneSecond)
{
    EXPECT_TRUE(hold(key, now, 900'000, true).empty());
    EXPECT_EQ(hold(key, now, 100'000, false), std::vector{KeyEvent::Short});
    EXPECT_EQ(hold(key, now, UserKey::long_press_us + (3 * tick), true), std::vector{KeyEvent::Long});
    EXPECT_TRUE(hold(key, now, 100'000, false).empty());
}

TEST_F(UserKeyTest, AHeldKeyRepeatsEveryTwoHundredMillisecondsAfterTheLongPress)
{
    // The press counts at its third sample, 20 ms in; repeats follow the long press every 0.2 s.
    const auto events = hold(key, now, UserKey::long_press_us + (3 * tick) + 600'000, true);
    EXPECT_EQ(events, (std::vector{KeyEvent::Long, KeyEvent::Repeat, KeyEvent::Repeat, KeyEvent::Repeat}));
}

TEST_F(UserKeyTest, AStuckKeyGivesOneLongPress)
{
    const auto events = hold(key, now, 30'000'000, true);
    ASSERT_FALSE(events.empty());
    EXPECT_EQ(std::count(events.begin(), events.end(), KeyEvent::Long), 1);
    EXPECT_EQ(std::count(events.begin(), events.end(), KeyEvent::Short), 0);
}

TEST(UserKey, AKeyPressedAtBootIsIgnoredUntilReleased)
{
    UserKey key;
    std::uint64_t now = 0;
    EXPECT_TRUE(hold(key, now, 3'000'000, true).empty());
    EXPECT_TRUE(hold(key, now, 100'000, false).empty());
    EXPECT_TRUE(hold(key, now, 100'000, true).empty());
    EXPECT_EQ(hold(key, now, 100'000, false), std::vector{KeyEvent::Short});
}

// The menu over a table of its own, sorted by name as the generator emits it.
struct Values {
    std::int32_t count = 4;
    float gain = 0.5F;
    std::int32_t lock = 0;
    std::int32_t mode = 0;
};
Values values;

constexpr std::array<std::int32_t, 2> lock_values{0, 1};
constexpr std::array<std::int32_t, 3> mode_values{0, 1, -1};
constexpr std::array<std::string_view, 2> lock_names{"OFF", "ON"};
constexpr std::array<std::string_view, 3> mode_names{"SLOW", "FAST", "OFF"};

ParamInfo int_info(std::string_view name, std::int32_t* value, std::int32_t def, std::int32_t min, std::int32_t max)
{
    return {.name = name,
            .hash = vk::core::param_hash(name),
            .type = ParamType::Int32,
            .flags = 0,
            .default_value = ParamValue::from_int(def),
            .min = ParamValue::from_int(min),
            .max = ParamValue::from_int(max),
            .enum_values = {},
            .bitmask = 0,
            .int_value = value,
            .float_value = nullptr};
}

std::array<ParamInfo, 4> make_table()
{
    ParamInfo count = int_info("M_COUNT", &values.count, 4, 0, 10);
    count.flags = vk::core::param_flags::reboot_required;
    const ParamInfo gain{.name = "M_GAIN",
                         .hash = vk::core::param_hash("M_GAIN"),
                         .type = ParamType::Float,
                         .flags = 0,
                         .default_value = ParamValue::from_float(0.5F),
                         .min = ParamValue::from_float(0.0F),
                         .max = ParamValue::from_float(1.0F),
                         .enum_values = {},
                         .bitmask = 0,
                         .int_value = nullptr,
                         .float_value = &values.gain};
    ParamInfo lock = int_info("M_LOCK", &values.lock, 0, 0, 1);
    lock.flags = vk::core::param_flags::locked_when_armed;
    lock.enum_values = lock_values;
    ParamInfo mode = int_info("M_MODE", &values.mode, 0, -1, 1);
    mode.enum_values = mode_values;
    return {count, gain, lock, mode};
}

const std::array<MenuParam, 4> menu_entries{{
    {.index = 0, .step = ParamValue::from_int(3), .value_names = {}},
    {.index = 1, .step = ParamValue::from_float(0.1F), .value_names = {}},
    {.index = 2, .step = {}, .value_names = lock_names},
    {.index = 3, .step = {}, .value_names = mode_names},
}};

void publish_armed(bool armed)
{
    vk::msg::ArmingState state{};
    state.armed = armed;
    vk::core::Publication<vk::msg::ArmingState>().publish(state);
}

class LocalMenuTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        values = Values{};
        publish_armed(false);
    }

    void TearDown() override { publish_armed(false); }

    void key(KeyEvent event, bool armed = false)
    {
        now += 100'000;
        menu.on_key(event, now, armed);
        menu.tick(now, armed);
    }

    // Opens the menu and edits the item at `position`.
    void edit(std::size_t position)
    {
        key(KeyEvent::Long);
        for (std::size_t i = 0; i < position; ++i) {
            key(KeyEvent::Short);
        }
        key(KeyEvent::Long);
        ASSERT_EQ(menu.state(), MenuState::Edit);
    }

    // Moves to `action` and runs it.
    void run(MenuAction action)
    {
        while (menu.action() != action) {
            key(KeyEvent::Short);
        }
        key(KeyEvent::Long);
    }

    [[nodiscard]] std::string text() const
    {
        TextScreen screen;
        vk::local_display::render_menu(menu, screen);
        std::string out;
        for (std::size_t line = 0; line < TextScreen::lines; ++line) {
            std::string_view row = screen.line(line);
            while (!row.empty() && row.back() == ' ') {
                row.remove_suffix(1);
            }
            out.append(row).push_back('\n');
        }
        return out;
    }

    std::array<ParamInfo, 4> table = make_table();
    vk::core::Parameters parameters{table};
    LocalMenu menu{parameters, menu_entries};
    std::uint64_t now = 0;
};

TEST_F(LocalMenuTest, ClosedAShortPressSwitchesPagesAndALongPressOpens)
{
    EXPECT_EQ(menu.page(), Page::Status);
    key(KeyEvent::Short);
    EXPECT_EQ(menu.page(), Page::Attitude);
    EXPECT_EQ(menu.state(), MenuState::Closed);
    key(KeyEvent::Long);
    EXPECT_EQ(menu.state(), MenuState::List);
    EXPECT_EQ(menu.item(), 0);
}

TEST_F(LocalMenuTest, ALongPressWhileArmedNeverOpensTheMenu)
{
    key(KeyEvent::Long, true);
    EXPECT_EQ(menu.state(), MenuState::Closed);
    key(KeyEvent::Short, true);
    EXPECT_EQ(menu.page(), Page::Attitude);
}

TEST_F(LocalMenuTest, TheListWrapsThroughExitAndExitCloses)
{
    key(KeyEvent::Long);
    for (int i = 0; i < 4; ++i) {
        key(KeyEvent::Short);
    }
    EXPECT_EQ(menu.item(), LocalMenu::no_item);
    EXPECT_EQ(menu.position(), 4U);
    key(KeyEvent::Short);
    EXPECT_EQ(menu.position(), 0U);
    for (int i = 0; i < 4; ++i) {
        key(KeyEvent::Short);
    }
    key(KeyEvent::Long);
    EXPECT_EQ(menu.state(), MenuState::Closed);
}

TEST_F(LocalMenuTest, ANumberStepsOnItsGridWithinTheRangeAndSaves)
{
    edit(1);  // M_GAIN, 0.5
    run(MenuAction::Increase);
    for (int i = 0; i < 10; ++i) {
        key(KeyEvent::Long);
    }
    EXPECT_FLOAT_EQ(menu.staged().as_float(), 1.0F);
    EXPECT_FLOAT_EQ(values.gain, 0.5F);  // nothing written yet
    run(MenuAction::Decrease);
    key(KeyEvent::Long);
    key(KeyEvent::Long);
    EXPECT_FLOAT_EQ(menu.staged().as_float(), 0.7F);
    run(MenuAction::Save);
    EXPECT_EQ(menu.state(), MenuState::List);
    EXPECT_FLOAT_EQ(values.gain, 0.7F);
    EXPECT_EQ(menu.last_result(), std::optional{ParamResult::Ok});
    EXPECT_EQ(menu.last_parameter(), 1);
}

TEST_F(LocalMenuTest, CancelWritesNothingAndDefaultStagesTheDefault)
{
    values.gain = 0.9F;
    edit(1);
    run(MenuAction::Decrease);
    run(MenuAction::Cancel);
    EXPECT_FLOAT_EQ(values.gain, 0.9F);
    key(KeyEvent::Long);
    run(MenuAction::Default);
    EXPECT_FLOAT_EQ(menu.staged().as_float(), 0.5F);
    run(MenuAction::Save);
    EXPECT_FLOAT_EQ(values.gain, 0.5F);
}

TEST_F(LocalMenuTest, AnIntegerStepsByItsStepAndClamps)
{
    edit(0);  // M_COUNT, 4, step 3, 0 to 10
    run(MenuAction::Increase);
    EXPECT_EQ(menu.staged().as_int(), 7);
    key(KeyEvent::Long);
    EXPECT_EQ(menu.staged().as_int(), 10);
    run(MenuAction::Decrease);
    for (int i = 0; i < 5; ++i) {
        key(KeyEvent::Long);
    }
    EXPECT_EQ(menu.staged().as_int(), 0);
}

TEST_F(LocalMenuTest, AnEnumStepsThroughItsValuesAndStopsAtTheEnds)
{
    edit(3);  // M_MODE: SLOW, FAST, OFF
    run(MenuAction::Increase);
    EXPECT_EQ(menu.staged().as_int(), 1);
    key(KeyEvent::Long);
    key(KeyEvent::Long);
    EXPECT_EQ(menu.staged().as_int(), -1);
    run(MenuAction::Save);
    EXPECT_EQ(values.mode, -1);
}

TEST_F(LocalMenuTest, OnlyPlusAndMinusRepeatAndSaveRunsOncePerPress)
{
    edit(0);
    run(MenuAction::Increase);
    key(KeyEvent::Repeat);
    EXPECT_EQ(menu.staged().as_int(), 10);
    while (menu.action() != MenuAction::Save) {
        key(KeyEvent::Short);
    }
    key(KeyEvent::Repeat);
    key(KeyEvent::Repeat);
    EXPECT_EQ(menu.state(), MenuState::Edit);
    EXPECT_EQ(values.count, 4);
    key(KeyEvent::Long);
    EXPECT_EQ(values.count, 10);
}

TEST_F(LocalMenuTest, ARejectedSaveShowsItsReason)
{
    // The parameter service sees the vehicle armed, as in the race between an
    // arm request and a long press.
    edit(2);
    run(MenuAction::Increase);
    publish_armed(true);
    run(MenuAction::Save);
    EXPECT_EQ(menu.last_result(), std::optional{ParamResult::LockedWhileArmed});
    EXPECT_EQ(values.lock, 0);
    EXPECT_NE(text().find("LOCKED"), std::string::npos);
}

TEST_F(LocalMenuTest, ThirtySecondsWithoutAKeyCloseTheMenuAndDiscardTheEdit)
{
    edit(1);
    run(MenuAction::Increase);
    menu.tick(now + LocalMenu::idle_close_us - 1, false);
    EXPECT_EQ(menu.state(), MenuState::Edit);
    menu.tick(now + LocalMenu::idle_close_us, false);
    EXPECT_EQ(menu.state(), MenuState::Closed);
    EXPECT_FLOAT_EQ(values.gain, 0.5F);
}

TEST_F(LocalMenuTest, ArmingClosesTheMenuAtOnce)
{
    edit(1);
    menu.tick(now + 1, true);
    EXPECT_EQ(menu.state(), MenuState::Closed);
}

TEST_F(LocalMenuTest, AChangedRebootRequiredParameterIsMarked)
{
    EXPECT_FALSE(menu.reboot_required());
    edit(0);
    run(MenuAction::Increase);
    run(MenuAction::Save);
    EXPECT_TRUE(menu.reboot_required());
    EXPECT_TRUE(menu.needs_reboot(0));
    EXPECT_FALSE(menu.needs_reboot(1));
}

TEST_F(LocalMenuTest, TheListShowsNamesAndTheSelectedValue)
{
    key(KeyEvent::Long);
    key(KeyEvent::Short);
    EXPECT_EQ(text(),
              "MENU\n"
              " M_COUNT\n"
              ">M_GAIN\n"
              " M_LOCK\n"
              " M_MODE\n"
              " EXIT\n"
              "\n"
              "= 0.5\n");
}

TEST_F(LocalMenuTest, AnEditShowsTheCurrentAndStagedValueAndTheAction)
{
    edit(3);
    run(MenuAction::Increase);
    // A Link changes the value meanwhile: the menu shows it and keeps the staged value.
    ASSERT_EQ(parameters.set("M_MODE", ParamValue::from_int(-1)), ParamResult::Ok);
    while (menu.action() != MenuAction::Save) {
        key(KeyEvent::Short);
    }
    EXPECT_EQ(text(),
              "EDIT\n"
              "M_MODE\n"
              "NOW OFF\n"
              "NEW FAST\n"
              "\n"
              "+ - DEF SAVE CANCEL\n"
              "        ^^^^\n"
              "SHORT: NEXT LONG: DO\n");
}

TEST(LocalMenuFormat, NumbersShowTheDecimalsOfTheirStep)
{
    std::int32_t integer = -12;
    float number = 0.05F;
    const ParamInfo int_param = int_info("F_INT", &integer, 0, -100, 100);
    ParamInfo float_param = int_param;
    float_param.type = ParamType::Float;
    const MenuParam hundredths{.index = 0, .step = ParamValue::from_float(0.01F), .value_names = {}};
    const MenuParam tenths{.index = 0, .step = ParamValue::from_float(0.1F), .value_names = {}};
    EXPECT_EQ(vk::local_display::format_value(int_param, hundredths, ParamValue::from_int(integer)).view(), "-12");
    EXPECT_EQ(vk::local_display::format_value(float_param, hundredths, ParamValue::from_float(number)).view(), "0.05");
    EXPECT_EQ(vk::local_display::format_value(float_param, tenths, ParamValue::from_float(1.0F)).view(), "1.0");
    EXPECT_EQ(vk::local_display::format_value(float_param, tenths, ParamValue::from_float(-2.25F)).view(), "-2.3");
}

}  // namespace
