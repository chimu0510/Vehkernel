// SPDX-License-Identifier: GPL-3.0-or-later
// Example 10-motor-outputs: Actuator control on outputs 1 to 4, output watchdog, and actuator power sense.
// ADR-0002 (bus backbone), ADR-0004 (FreeRTOS), ADR-0005 (setpoint arbitration, arming, output watchdog),
// ADR-0008 (bus and work queues), ADR-0009 (layered HAL), ADR-0013 (control chain and actuators),
// ADR-0022 (local display), ADR-0025 (board examples), spec 0006 (first wheel motion), spec 0013 (board examples).
// Lesson: examples/c50c-v1_1/10-motor-outputs/README.md
#include <line.hpp>
#include <support.hpp>
#include <vk/core/bus.hpp>
#include <vk/core/static_instance.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/drivers/ssd1306.hpp>
#include <vk/local_display/pages.hpp>
#include <vk/msg/actuator_demand.hpp>
#include <vk/msg/actuator_outputs.hpp>
#include <vk/msg/arming_state.hpp>
#include <vk/params/params.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace {

using example::Line;

// Maximum duty cap enforced in this example (spec 0013, ADR-0005).
// Lowered in vk::params::mot.out_max in RAM before starting ActuatorTask so the
// speed loop PI term clamps output duty to 20 % in either direction.
constexpr float max_duty = 0.2F;

// Full speed at 100 % duty (rad/s), matching the motor model in sim/src/motor.cpp
// and the speed loop's feedforward gain MOT_SPD_FF = 0.02 s/rad (1.0 / 0.02 = 50.0).
constexpr float full_scale_speed_rad_s = 50.0F;

// Fixed list of demand steps cycled by each press of user key S3:
// 0 %, +10 %, +20 %, -10 %, -20 %.
// Step 0 is 0 % (initial state / stopped).
// Each key press advances to the next step, wrapping from -20 % back to 0 %.
constexpr std::array<float, 5> duty_steps{{
    0.00F,   // Step 0: 0 % (initial state / stopped)
    0.10F,   // Step 1: +10 %
    0.20F,   // Step 2: +20 % (capped at max_duty)
    -0.10F,  // Step 3: -10 %
    -0.20F,  // Step 4: -20 % (capped at max_duty)
}};

// Plain debouncer: samples every 10 ms. A candidate level counts after
// debounce_count consecutive agreeing samples (30 ms), as in 02-user-key.
class Debouncer {
public:
    static constexpr unsigned debounce_count = 3;

    bool update(bool raw_pressed)
    {
        if (raw_pressed == candidate_) {
            if (agreeing_ < debounce_count) {
                ++agreeing_;
                if (agreeing_ == debounce_count && debounced_ != candidate_) {
                    debounced_ = candidate_;
                    return debounced_;
                }
            }
        } else {
            candidate_ = raw_pressed;
            agreeing_ = 1;
        }
        return false;
    }

private:
    bool candidate_ = false;
    bool debounced_ = false;
    unsigned agreeing_ = 0;
};

// Work item on wq:ctrl running at the base control rate (200 Hz, 5000 us).
// Debounces user key S3, checks actuator power, feeds actuator_demand to
// ActuatorTask, and publishes arming_state in place of the commander.
class DemandItem final : public vk::core::WorkItem {
public:
    static constexpr std::uint32_t interval_us = 5'000;  // 200 Hz control cycle

    explicit DemandItem(vk::core::WorkQueue& queue)
        : WorkItem("demand", queue)
    {
    }

    void start()
    {
        example::report("demand", 0.0F);
        schedule_periodic(interval_us);
    }

    [[nodiscard]] std::size_t step_index() const { return step_index_; }
    [[nodiscard]] float commanded_duty() const { return commanded_duty_; }

protected:
    void run() override
    {
        const std::uint64_t now = queue().now_us();
        const bool power = example::actuator_power();

        // Sample debouncer every 10 ms (every two 5 ms runs).
        ++debounce_tick_;
        if (debounce_tick_ >= 2) {
            debounce_tick_ = 0;
            if (debouncer_.update(example::user_key_pressed())) {
                if (power) {
                    step_index_ = (step_index_ + 1) % duty_steps.size();
                    commanded_duty_ = std::clamp(duty_steps[step_index_], -max_duty, max_duty);
                    example::report("demand", commanded_duty_);
                }
            }
        }

        // Safety: power cut disarms immediately and resets demand to 0.
        // Nothing in the example may drive an output when drive power reads cut.
        if (!power && (armed_ || step_index_ != 0)) {
            step_index_ = 0;
            commanded_duty_ = 0.0F;
            set_armed(now, false);
            example::report("demand", 0.0F);
            return;
        }

        // While a non-zero demand is held and actuator power is present:
        // keep vehicle armed and publish actuator_demand every control cycle.
        if (power && commanded_duty_ != 0.0F) {
            set_armed(now, true);
            const float target_speed = commanded_duty_ * full_scale_speed_rad_s;
            vk::msg::ActuatorDemand demand{};
            demand.timestamp = now;
            demand.timestamp_setpoint = now;
            demand.wheel_speed = {target_speed, 0.0F, 0.0F, 0.0F};
            demand_pub_.publish(demand);
        } else {
            // When demand stops (step 0 or back to 0):
            // Disarm the actuator task so outputs stop immediately.
            set_armed(now, false);
        }
    }

private:
    void set_armed(std::uint64_t now, bool armed)
    {
        if (armed_ != armed) {
            armed_ = armed;
            vk::msg::ArmingState arming{};
            arming.timestamp = now;
            arming.armed = armed;
            arming_pub_.publish(arming);
        }
    }

    Debouncer debouncer_{};
    std::uint8_t debounce_tick_ = 0;
    std::size_t step_index_ = 0;
    float commanded_duty_ = 0.0F;
    bool armed_ = false;

    vk::core::Publication<vk::msg::ArmingState> arming_pub_;
    vk::core::Publication<vk::msg::ActuatorDemand> demand_pub_;
};

// Work item on wq:slow updating the OLED and reporting at 4 Hz (250 ms).
class DisplayItem final : public vk::core::WorkItem {
public:
    static constexpr std::uint32_t interval_us = 250'000;  // 4 Hz

    DisplayItem(const DemandItem& demand_item, vk::drivers::Ssd1306& display, vk::core::WorkQueue& queue)
        : WorkItem("display", queue), demand_item_(demand_item), display_(display)
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

        vk::msg::ActuatorOutputs outputs{};
        while (outputs_sub_.copy(outputs)) {
            last_outputs_ = outputs;
        }

        const bool power = example::actuator_power();
        const auto step = static_cast<std::uint32_t>(demand_item_.step_index());
        const float cmd_duty = demand_item_.commanded_duty();
        const float out1 = last_outputs_.output[0];

        // Report applied outputs and power state at 4 Hz for SITL L3 check.
        example::report("out1", out1);
        example::report("out2", last_outputs_.output[1]);
        example::report("out3", last_outputs_.output[2]);
        example::report("out4", last_outputs_.output[3]);
        example::report("power", power ? 1 : 0);

        // Update SSD1306 OLED display.
        screen_.clear();
        screen_.put(0, 0, "VEHKERNEL 10-MOTORS");
        screen_.put(2, 0, Line().add("STEP: ").add(step).view());
        screen_.put(3, 0, Line().add("DEMAND: ").add_fixed(cmd_duty).view());
        screen_.put(4, 0, Line().add("OUTPUT 1: ").add_fixed(out1).view());
        screen_.put(5, 0, Line().add("POWER: ").add(power ? "ON" : "OFF").view());
        screen_.draw(display_.frame());

        static_cast<void>(display_.update(now, vk::drivers::MonoFrame::pages));
    }

private:
    const DemandItem& demand_item_;
    vk::drivers::Ssd1306& display_;
    vk::local_display::TextScreen screen_{};
    vk::core::Subscription<vk::msg::ActuatorOutputs> outputs_sub_{0};
    vk::msg::ActuatorOutputs last_outputs_{};
};

vk::core::StaticInstance<DemandItem> demand_item;
vk::core::StaticInstance<DisplayItem> display_item;

}  // namespace

int main(int argc, char** argv)
{
    example::init(argc, argv);

    // Lower MOT_OUT_MAX in RAM before the actuator task starts (spec 0013, ADR-0005).
    // The actuator control speed loop clamps its output duty to MOT_OUT_MAX on every run;
    // lowering it to max_duty ensures the PI term can never drive duty beyond 20 %.
    // This RAM value is never stored: example::start_parameters() is not called, so nothing
    // writes parameters to Flash. For the same reason, the example runs with the parameters'
    // compiled-in defaults rather than board-stored calibrations (e.g. motor reversal, encoder CPR,
    // speed loop gains), so a motor may turn the other way than under the firmware;
    // the duty cap holds in either direction.
    vk::params::mot.out_max = std::min(vk::params::mot.out_max, max_duty);

    example::start_actuators();

    auto& display = example::display();
    auto& ctrl_queue = vk::core::work_queue(vk::core::WorkQueueTier::Ctrl);
    auto& slow_queue = vk::core::work_queue(vk::core::WorkQueueTier::Slow);

    demand_item.emplace(ctrl_queue);
    display_item.emplace(*demand_item, display, slow_queue);

    demand_item->start();
    display_item->start();

    example::start_scheduler();

    return 0;
}
