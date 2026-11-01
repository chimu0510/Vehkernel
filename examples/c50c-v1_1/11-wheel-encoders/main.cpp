// SPDX-License-Identifier: GPL-3.0-or-later
// Example 11-wheel-encoders: Quadrature encoders for motor outputs 1 to 4 and wheel_encoders.
// ADR-0002 (bus backbone), ADR-0004 (FreeRTOS), ADR-0005 (actuators and arming),
// ADR-0008 (bus and work queues), ADR-0009 (layered HAL), ADR-0013 (control chain and wheel encoders),
// ADR-0022 (local display), ADR-0025 (board examples), spec 0006 (first wheel motion), spec 0013 (board examples).
// Lesson: examples/c50c-v1_1/11-wheel-encoders/README.md
#include <line.hpp>
#include <support.hpp>
#include <vk/core/bus.hpp>
#include <vk/core/static_instance.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/drivers/ssd1306.hpp>
#include <vk/local_display/pages.hpp>
#include <vk/msg/wheel_encoders.hpp>
#include <vk/params/params.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>

namespace {

using example::Line;

// Work item on wq:slow updating the OLED and reporting at 4 Hz (250 ms).
// Drains the wheel_encoders subscription, keeps the newest message, computes
// signed counts from angle and MOTn_CPR, reports and draws on the OLED.
class EncoderDisplayItem final : public vk::core::WorkItem {
public:
    static constexpr std::uint32_t interval_us = 250'000;  // 4 Hz

    EncoderDisplayItem(vk::drivers::Ssd1306& display, vk::core::WorkQueue& queue)
        : WorkItem("encoders_display", queue), display_(display)
    {
    }

    void start()
    {
        schedule_periodic(interval_us);
    }

protected:
    void run() override
    {
        const std::uint64_t now = queue().now_us();

        vk::msg::WheelEncoders encoders{};
        while (encoders_sub_.copy(encoders)) {
            last_encoders_ = encoders;
        }

        // MOTn_CPR parameter defaults: 36000 on C50C V1.1 (500 lines * 4 * 18 gearbox),
        // 20000 in SITL. Parameter storage is not started, so compiled-in defaults apply.
        const std::array<float, 4> cpr{{
            vk::params::mot.mot1_cpr,
            vk::params::mot.mot2_cpr,
            vk::params::mot.mot3_cpr,
            vk::params::mot.mot4_cpr,
        }};

        // Signed count = round(angle / (2 * pi) * CPR).
        std::array<std::int32_t, 4> counts{};
        for (std::size_t i = 0; i < 4; ++i) {
            counts[i] = static_cast<std::int32_t>(
                std::lround(last_encoders_.angle[i] / (2.0F * std::numbers::pi_v<float>) * cpr[i]));
        }

        // Report timestamp (us), wheel speeds (float), and signed counts (signed integer)
        // at 4 Hz for SITL L3 check.
        example::report("encoders_us", last_encoders_.timestamp);
        example::report("speed1", last_encoders_.speed[0]);
        example::report("speed2", last_encoders_.speed[1]);
        example::report("speed3", last_encoders_.speed[2]);
        example::report("speed4", last_encoders_.speed[3]);
        example::report("count1", counts[0]);
        example::report("count2", counts[1]);
        example::report("count3", counts[2]);
        example::report("count4", counts[3]);

        // Update SSD1306 OLED display:
        // Line 0: title
        // Line 1: blank
        // Lines 2 to 5: one line per wheel (number, signed count, signed speed)
        screen_.clear();
        screen_.put(0, 0, "VEHKERNEL 11-ENCODERS");
        screen_.put(2, 0, Line().add("1 ").add_signed(counts[0]).add(" ").add_fixed(last_encoders_.speed[0]).view());
        screen_.put(3, 0, Line().add("2 ").add_signed(counts[1]).add(" ").add_fixed(last_encoders_.speed[1]).view());
        screen_.put(4, 0, Line().add("3 ").add_signed(counts[2]).add(" ").add_fixed(last_encoders_.speed[2]).view());
        screen_.put(5, 0, Line().add("4 ").add_signed(counts[3]).add(" ").add_fixed(last_encoders_.speed[3]).view());
        screen_.draw(display_.frame());

        static_cast<void>(display_.update(now, vk::drivers::MonoFrame::pages));
    }

private:
    vk::drivers::Ssd1306& display_;
    vk::local_display::TextScreen screen_{};
    vk::core::Subscription<vk::msg::WheelEncoders> encoders_sub_{0};
    vk::msg::WheelEncoders last_encoders_{};
};

vk::core::StaticInstance<EncoderDisplayItem> display_item;

}  // namespace

int main(int argc, char** argv)
{
    example::init(argc, argv);

    // Start actuator task and encoder reading (spec 0013, ADR-0013).
    // On the board, the example never drives an output: it never publishes arming_state
    // or actuator_demand. ActuatorTask remains disarmed (duty stays 0), and still reads
    // the quadrature encoders and publishes wheel_encoders on every control cycle (200 Hz).
    // Parameter storage is not started, so parameter defaults apply (MOTn_CPR = 36000 on C50C,
    // 20000 in SITL). MOTn_ENC_REV from Flash does not apply.
    example::start_actuators();

    auto& display = example::display();
    auto& slow_queue = vk::core::work_queue(vk::core::WorkQueueTier::Slow);

    display_item.emplace(display, slow_queue);
    display_item->start();

    example::start_scheduler();

    return 0;
}
