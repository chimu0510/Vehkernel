// SPDX-License-Identifier: GPL-3.0-or-later
// The SSD1306 driver against the simulated controller (ADR-0022): reset and
// the vendor firmware's initialisation, the frame's coordinates on the panel,
// and writing only changed pages, at most two per update.
#include <vk/drivers/ssd1306.hpp>
#include <vk/hal/display_bus.hpp>
#include <vk/sim/ssd1306.hpp>

#include <gtest/gtest.h>

#include <bit>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace {

using vk::drivers::MonoFrame;
using vk::drivers::Ssd1306;

static_assert(vk::hal::DisplayBus<vk::sim::Ssd1306>);

class Ssd1306Test : public ::testing::Test {
protected:
    // Runs updates until the driver has written every page once.
    void bring_up()
    {
        driver.update(0);
        driver.update(Ssd1306::reset_time_us);
        for (int i = 0; i < 3; ++i) {
            driver.update(Ssd1306::reset_time_us);
        }
        panel.take_written_pages();
    }

    vk::sim::Ssd1306 panel;
    Ssd1306 driver{vk::hal::DisplayDevice(panel)};
};

TEST_F(Ssd1306Test, HoldsResetFor100MsThenSendsTheVendorSequence)
{
    EXPECT_EQ(driver.update(0), 0U);
    EXPECT_EQ(driver.state(), Ssd1306::State::Resetting);
    EXPECT_EQ(driver.update(Ssd1306::reset_time_us - 1), 0U);
    EXPECT_FALSE(panel.displaying());
    EXPECT_TRUE(panel.commands().empty());

    driver.update(Ssd1306::reset_time_us);
    EXPECT_EQ(driver.state(), Ssd1306::State::Ready);
    EXPECT_TRUE(panel.displaying());
    // WHEELTEC_C50X 2026.05.29, oled.c, OLED_Init, then the first page address.
    const std::vector<std::uint8_t> vendor{0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0x8D,
                                           0x14, 0x20, 0x02, 0xA1, 0xC0, 0xDA, 0x12, 0x81, 0xEF,
                                           0xD9, 0xF1, 0xDB, 0x30, 0xA4, 0xA6, 0xAF};
    ASSERT_GE(panel.commands().size(), vendor.size());
    EXPECT_EQ(std::vector<std::uint8_t>(panel.commands().begin(),
                                        std::next(panel.commands().begin(), static_cast<std::ptrdiff_t>(vendor.size()))),
              vendor);
}

TEST_F(Ssd1306Test, WritesEveryPageAfterInitialisationTwoAtATime)
{
    driver.update(0);
    EXPECT_EQ(driver.update(Ssd1306::reset_time_us), 2U);
    EXPECT_EQ(driver.update(Ssd1306::reset_time_us), 2U);
    EXPECT_EQ(driver.update(Ssd1306::reset_time_us), 2U);
    EXPECT_EQ(driver.update(Ssd1306::reset_time_us), 2U);
    EXPECT_EQ(panel.take_written_pages(), 0xFF);
    EXPECT_EQ(driver.update(Ssd1306::reset_time_us), 0U);
    EXPECT_EQ(panel.take_written_pages(), 0);
}

TEST_F(Ssd1306Test, FramePixelsAppearAtTheSameImageCoordinates)
{
    bring_up();
    MonoFrame& frame = driver.frame();
    frame.set(0, 0, true);
    frame.set(127, 63, true);
    frame.set(10, 20, true);
    frame.set(64, 7, true);
    while (driver.update(Ssd1306::reset_time_us) > 0) {
    }
    for (std::size_t y = 0; y < MonoFrame::height; ++y) {
        for (std::size_t x = 0; x < MonoFrame::width; ++x) {
            EXPECT_EQ(panel.pixel(x, y), frame.get(x, y)) << x << "," << y;
        }
    }
    EXPECT_TRUE(panel.pixel(10, 20));
}

TEST_F(Ssd1306Test, WritesOnlyChangedPagesAtMostTwoPerUpdate)
{
    bring_up();
    MonoFrame& frame = driver.frame();
    frame.set(5, 24, true);  // frame page 3: RAM page 4
    EXPECT_EQ(driver.update(Ssd1306::reset_time_us), 1U);
    EXPECT_EQ(panel.take_written_pages(), 1U << 4U);

    frame.set(5, 0, true);  // frame pages 0, 5 and 7
    frame.set(5, 40, true);
    frame.set(5, 56, true);
    EXPECT_EQ(driver.update(Ssd1306::reset_time_us), 2U);
    EXPECT_EQ(driver.update(Ssd1306::reset_time_us), 1U);
    EXPECT_EQ(panel.take_written_pages(), (1U << 7U) | (1U << 2U) | (1U << 0U));
    EXPECT_EQ(driver.update(Ssd1306::reset_time_us), 0U);
}

TEST_F(Ssd1306Test, AFrameSetBackIsNotWrittenAgain)
{
    bring_up();
    driver.frame().set(1, 1, true);
    driver.frame().set(1, 1, false);
    EXPECT_EQ(driver.update(Ssd1306::reset_time_us), 0U);
}

TEST(MonoFrame, IgnoresPixelsOutsideTheFrame)
{
    MonoFrame frame;
    frame.set(128, 0, true);
    frame.set(0, 64, true);
    EXPECT_EQ(frame, MonoFrame{});
    EXPECT_FALSE(frame.get(500, 500));
}

TEST(SimSsd1306, ShowsNothingUntilTheChargePumpAndDisplayAreOn)
{
    vk::sim::Ssd1306 panel;
    panel.set_reset(false);
    const std::vector<std::uint8_t> on{0xAF};
    panel.write_commands(on);
    EXPECT_FALSE(panel.displaying());
    const std::vector<std::uint8_t> pump{0x8D, 0x14};
    panel.write_commands(pump);
    EXPECT_TRUE(panel.displaying());
}

TEST(SimSsd1306, WritesItsImageAsAPbmFile)
{
    vk::sim::Ssd1306 panel;
    Ssd1306 driver{vk::hal::DisplayDevice(panel)};
    const auto path = std::filesystem::temp_directory_path() / "vk_ssd1306_test.pbm";
    panel.write_image_to(path.string());
    driver.frame().set(0, 0, true);
    driver.update(0);
    driver.update(Ssd1306::reset_time_us);

    std::ifstream file(path, std::ios::binary);
    const std::string image((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    EXPECT_EQ(image, panel.pbm());
    ASSERT_EQ(image.substr(0, 10), "P4\n128 64\n");
    EXPECT_EQ(image.size(), 10U + (128U / 8U * 64U));
    EXPECT_EQ(static_cast<unsigned char>(image.at(10)), 0x80U);  // the top left pixel
    std::filesystem::remove(path);
}

}  // namespace
