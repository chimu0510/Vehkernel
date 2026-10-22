// The local display's pages (ADR-0022): the text each page shows for given
// topic values, the image the panel shows, against golden PBM files, and the
// work item's timing on a fake clock: a key press on the panel within one run,
// topic values at most 5 Hz.
// Run with VK_UPDATE_GOLDEN=1 to rewrite the images after an intended change,
// then review them.
#include <vk/core/bus.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/drivers/ssd1306.hpp>
#include <vk/hal/display_bus.hpp>
#include <vk/local_display/font.hpp>
#include <vk/local_display/local_display.hpp>
#include <vk/local_display/local_menu.hpp>
#include <vk/local_display/pages.hpp>
#include <vk/local_display/user_key.hpp>
#include <vk/params/params.hpp>
#include <vk/sim/ssd1306.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <numbers>
#include <string>
#include <string_view>

namespace {

using vk::local_display::Inputs;
using vk::local_display::Page;
using vk::local_display::render;
using vk::local_display::TextScreen;

Inputs sample_inputs()
{
    Inputs inputs;
    vk::msg::VehicleStatus status{};
    status.mode = 1;
    status.armed = false;
    status.board_verified = true;
    status.emergency_stop = true;
    status.actuator_power = 2;
    status.skipped_checks = 1;
    status.arming_refusal = 2;
    status.disarm_reason = 3;
    status.output_watchdog_trips = 7;
    inputs.status = status;

    // Roll 10, pitch -20.5 and yaw 135 degrees, ZYX.
    constexpr float rad = std::numbers::pi_v<float> / 180.0F;
    const float cr = std::cos(10.0F * rad / 2), sr = std::sin(10.0F * rad / 2);
    const float cp = std::cos(-20.5F * rad / 2), sp = std::sin(-20.5F * rad / 2);
    const float cy = std::cos(135.0F * rad / 2), sy = std::sin(135.0F * rad / 2);
    vk::msg::VehicleAttitude attitude{};
    attitude.q = {(cr * cp * cy) + (sr * sp * sy), (sr * cp * cy) - (cr * sp * sy), (cr * sp * cy) + (sr * cp * sy),
                  (cr * cp * sy) - (sr * sp * cy)};
    attitude.accel_rejected = 42;
    inputs.attitude = attitude;

    vk::msg::LinkStatus link{};
    link.handshake_state = 1;
    link.rate_scale = 61;
    link.drops = 3;
    link.lost = 12;
    inputs.links.at(0) = link;
    return inputs;
}

std::string text_of(Page page, const Inputs& inputs)
{
    TextScreen screen;
    render(page, inputs, screen);
    std::string text;
    for (std::size_t line = 0; line < TextScreen::lines; ++line) {
        std::string_view row = screen.line(line);
        while (!row.empty() && row.back() == ' ') {
            row.remove_suffix(1);
        }
        text.append(row).push_back('\n');
    }
    return text;
}

TEST(LocalDisplayPages, StatusShowsTheVehicleStatus)
{
    EXPECT_EQ(text_of(Page::Status, sample_inputs()),
              "STATUS            1/3\n"
              "TEST DISARMED\n"
              "ESTOP ON  BOARD OK\n"
              "POWER WAIT\n"
              "SKIP ACT_POWER\n"
              "REFUSED ESTOP\n"
              "DISARM  POWER_LOST\n"
              "WATCHDOG 7\n");
}

TEST(LocalDisplayPages, StatusWithoutAStatusSaysSo)
{
    EXPECT_EQ(text_of(Page::Status, Inputs{}), "STATUS            1/3\nNO STATUS\n\n\n\n\n\n\n");
}

TEST(LocalDisplayPages, AttitudeShowsDegreesWithOneDecimal)
{
    EXPECT_EQ(text_of(Page::Attitude, sample_inputs()),
              "ATTITUDE          2/3\n"
              "\n"
              "ROLL         10.0 DEG\n"
              "PITCH       -20.5 DEG\n"
              "YAW         135.0 DEG\n"
              "\n"
              "ACC REJECTED 42\n"
              "\n");
}

TEST(LocalDisplayPages, LinkShowsEachInstance)
{
    EXPECT_EQ(text_of(Page::Link, sample_inputs()),
              "LINK              3/3\n"
              "LINK0 CONNECTED   61%\n"
              " DROPS 3\n"
              " LOST 12\n"
              "LINK1 NONE\n"
              "\n"
              "\n"
              "\n");
}

TEST(LocalDisplayPages, PagesFollowEachOtherInTurn)
{
    EXPECT_EQ(vk::local_display::next_page(Page::Status), Page::Attitude);
    EXPECT_EQ(vk::local_display::next_page(Page::Attitude), Page::Link);
    EXPECT_EQ(vk::local_display::next_page(Page::Link), Page::Status);
}

TEST(LocalDisplayPages, TextIsCutAtTheRightEdge)
{
    TextScreen screen;
    screen.put(0, 18, "ABCDEF");
    screen.put(9, 0, "ignored");
    EXPECT_EQ(screen.line(0).substr(18), "ABC");
    screen.put_right(1, "0123456789012345678901234");
    EXPECT_EQ(screen.line(1), "456789012345678901234");
}

TEST(LocalDisplayFont, UnknownCharactersDrawAsAQuestionMark)
{
    EXPECT_EQ(&vk::local_display::glyph('\x7F'), &vk::local_display::glyph('?'));
    EXPECT_EQ(&vk::local_display::glyph('\n'), &vk::local_display::glyph('?'));
}

// The image the panel shows once the driver has written every page.
std::string panel_image(Page page, const Inputs& inputs)
{
    vk::sim::Ssd1306 panel;
    vk::drivers::Ssd1306 driver{vk::hal::DisplayDevice(panel)};
    TextScreen screen;
    render(page, inputs, screen);
    screen.draw(driver.frame());
    driver.update(0);
    for (int i = 0; i < 4; ++i) {
        driver.update(vk::drivers::Ssd1306::reset_time_us);
    }
    return panel.pbm();
}

class LocalDisplayImages : public ::testing::TestWithParam<std::pair<Page, std::string_view>> {};

TEST_P(LocalDisplayImages, MatchTheGoldenImage)
{
    const auto [page, name] = GetParam();
    const std::string image = panel_image(page, sample_inputs());
    const std::filesystem::path golden = std::filesystem::path(VK_GOLDEN_DIR) / (std::string(name) + ".pbm");
    const char* update = std::getenv("VK_UPDATE_GOLDEN");
    if (update != nullptr && std::string_view(update) == "1") {
        std::filesystem::create_directories(golden.parent_path());
        std::ofstream(golden, std::ios::binary) << image;
    }
    std::ifstream file(golden, std::ios::binary);
    ASSERT_TRUE(file) << "missing golden image " << golden << "; run with VK_UPDATE_GOLDEN=1";
    const std::string expected((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    EXPECT_EQ(image, expected) << golden;
}

INSTANTIATE_TEST_SUITE_P(Pages, LocalDisplayImages,
                         ::testing::Values(std::pair{Page::Status, std::string_view("status")},
                                           std::pair{Page::Attitude, std::string_view("attitude")},
                                           std::pair{Page::Link, std::string_view("link")}),
                         [](const auto& test) { return std::string(test.param.second); });

std::uint64_t fake_now = 0;

std::uint64_t fake_clock() noexcept
{
    return fake_now;
}

// The work item with a local menu, driving a simulated panel, fed the sample topics.
class LocalDisplayWorkItemTest : public ::testing::Test {
protected:
    using LocalDisplay = vk::local_display::LocalDisplay;

    void SetUp() override
    {
        fake_now = 1'000'000;
        publish(sample_inputs());
        display.start();
        step(400'000);  // reset, initialisation and the first screen
        panel.take_written_pages();
    }

    void publish(const Inputs& inputs)
    {
        if (inputs.status) {
            status.publish(*inputs.status);
        }
        if (inputs.attitude) {
            attitude.publish(*inputs.attitude);
        }
        if (const auto& first_link = inputs.links.at(0)) {
            link.publish(*first_link);
        }
    }

    // Runs the queue every run interval for `duration_us`; returns the runs that wrote the panel.
    int step(std::uint64_t duration_us, const Inputs* changing = nullptr)
    {
        int writes = 0;
        for (std::uint64_t end = fake_now + duration_us; fake_now < end;) {
            fake_now += LocalDisplay::interval_us;
            if (changing != nullptr) {
                Inputs inputs = *changing;
                if (inputs.attitude) {
                    inputs.attitude->accel_rejected = static_cast<std::uint32_t>(fake_now / LocalDisplay::interval_us);
                }
                publish(inputs);
            }
            queue.process();
            writes += panel.take_written_pages() != 0 ? 1 : 0;
        }
        return writes;
    }

    vk::core::WorkQueue queue{"wq:test", 1, &fake_clock};
    vk::sim::Ssd1306 panel;
    vk::drivers::Ssd1306 driver{vk::hal::DisplayDevice(panel)};
    vk::local_display::LocalMenu menu{vk::params::parameters(), {}};
    LocalDisplay display{driver, &menu, queue};
    vk::core::Publication<vk::msg::VehicleStatus> status{0};
    vk::core::Publication<vk::msg::VehicleAttitude> attitude{0};
    vk::core::Publication<vk::msg::LinkStatus> link{0};
};

TEST_F(LocalDisplayWorkItemTest, AKeyPressIsWhollyOnThePanelAfterOneRun)
{
    menu.on_key(vk::local_display::KeyEvent::Short, fake_now, false);
    step(LocalDisplay::interval_us);
    EXPECT_EQ(panel.pbm(), panel_image(Page::Attitude, sample_inputs()));
}

TEST_F(LocalDisplayWorkItemTest, TopicValuesChangeAtMostFiveTimesASecond)
{
    menu.on_key(vk::local_display::KeyEvent::Short, fake_now, false);
    step(LocalDisplay::interval_us);
    panel.take_written_pages();

    // accel_rejected, on the attitude page, changes before every run.
    const Inputs inputs = sample_inputs();
    const int writes = step(1'000'000, &inputs);
    EXPECT_GE(writes, 4);
    EXPECT_LE(writes, 5);
}

}  // namespace
