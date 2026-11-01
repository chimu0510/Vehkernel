// SPDX-License-Identifier: GPL-3.0-or-later
// Example 12-usb-gamepad: The USB host on CN8 and an XInput gamepad, publishing manual_control_input.
// ADR-0002 (bus backbone), ADR-0008 (bus and work queues), ADR-0009 (layered HAL), ADR-0022 (local display),
// ADR-0025 (board examples), spec 0010 (remote control), spec 0013 (board examples).
// Lesson: examples/c50c-v1_1/12-usb-gamepad/README.md
#include <line.hpp>
#include <support.hpp>
#include <vk/core/bus.hpp>
#include <vk/core/static_instance.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/drivers/ssd1306.hpp>
#include <vk/local_display/pages.hpp>
#include <vk/msg/enums.hpp>
#include <vk/msg/manual_control_input.hpp>

#include <array>
#include <cstdint>
#include <string_view>

namespace {

using example::Line;
using vk::msg::GamepadButton;

struct ButtonName {
    GamepadButton button;
    std::string_view name;
};

// The buttons as the screen names them, on two lines that fit its 21 columns
// with every button held.
constexpr std::array<ButtonName, 8> front_buttons{{
    {GamepadButton::A, "A "},
    {GamepadButton::B, "B "},
    {GamepadButton::X, "X "},
    {GamepadButton::Y, "Y "},
    {GamepadButton::LeftShoulder, "LB "},
    {GamepadButton::RightShoulder, "RB "},
    {GamepadButton::Start, "ST "},
    {GamepadButton::Back, "BK "},
}};
constexpr std::array<ButtonName, 7> other_buttons{{
    {GamepadButton::DpadUp, "U "},
    {GamepadButton::DpadDown, "D "},
    {GamepadButton::DpadLeft, "L "},
    {GamepadButton::DpadRight, "R "},
    {GamepadButton::LeftStick, "LS "},
    {GamepadButton::RightStick, "RS "},
    {GamepadButton::Guide, "GD "},
}};

// The names of the buttons held, among `names`.
template <std::size_t N>
Line pressed(std::uint16_t buttons, const std::array<ButtonName, N>& names)
{
    Line line;
    for (const ButtonName& entry : names) {
        if ((buttons & (1U << static_cast<std::uint8_t>(entry.button))) != 0U) {
            line.add(entry.name);
        }
    }
    return line;
}

// Work item on wq:slow drawing the newest manual_control_input at 10 Hz.
class GamepadDisplayItem final : public vk::core::WorkItem {
public:
    static constexpr std::uint32_t interval_us = 100'000;  // 10 Hz

    GamepadDisplayItem(vk::drivers::Ssd1306& display, vk::core::WorkQueue& queue)
        : WorkItem("gamepad_display", queue), display_(display)
    {
    }

    void start()
    {
        schedule_periodic(interval_us);
    }

protected:
    void run() override
    {
        vk::msg::ManualControlInput input{};
        while (input_sub_.copy(input)) {
            last_input_ = input;
        }

        screen_.clear();
        screen_.put(0, 0, "VEHKERNEL 12-GAMEPAD");
        screen_.put(2, 0, last_input_.connected ? "PAD: CONNECTED" : "PAD: NONE");
        screen_.put(3, 0, Line().add("LEFT  ").add_fixed(last_input_.sticks[0]).add(" ")
                              .add_fixed(last_input_.sticks[1]).view());
        screen_.put(4, 0, Line().add("RIGHT ").add_fixed(last_input_.sticks[2]).add(" ")
                              .add_fixed(last_input_.sticks[3]).view());
        screen_.put(5, 0, Line().add("TRIG  ").add_float(last_input_.triggers[0]).add(" ")
                              .add_float(last_input_.triggers[1]).view());
        screen_.put(6, 0, pressed(last_input_.buttons, front_buttons).view());
        screen_.put(7, 0, pressed(last_input_.buttons, other_buttons).view());
        screen_.draw(display_.frame());

        static_cast<void>(display_.update(queue().now_us(), vk::drivers::MonoFrame::pages));
    }

private:
    vk::drivers::Ssd1306& display_;
    vk::local_display::TextScreen screen_{};
    vk::core::Subscription<vk::msg::ManualControlInput> input_sub_{0};
    vk::msg::ManualControlInput last_input_{};
};

vk::core::StaticInstance<GamepadDisplayItem> display_item;

}  // namespace

int main(int argc, char** argv)
{
    example::init(argc, argv);

    // The USB host's own task, and a work item on wq:io that publishes manual_control_input (spec 0010).
    example::start_usb_gamepad();

    display_item.emplace(example::display(), vk::core::work_queue(vk::core::WorkQueueTier::Slow));
    display_item->start();

    example::start_scheduler();

    return 0;
}
