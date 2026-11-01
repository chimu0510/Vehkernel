// SPDX-License-Identifier: GPL-3.0-or-later
// Example 06-imu: ICM20948 IMU driver publishing imu_sample over software I2C; OLED shows specific force, rate and down axis.
// ADR-0002 (bus backbone), ADR-0004 (FreeRTOS), ADR-0006 (schema), ADR-0008 (bus and work queues), ADR-0009 (layered HAL), ADR-0014 (estimation architecture), ADR-0015 (selectable timing), ADR-0022 (local display), ADR-0025 (board examples).
// Lesson: examples/c50c-v1_1/06-imu/README.md
#include <line.hpp>
#include <support.hpp>
#include <vk/core/bus.hpp>
#include <vk/core/critical_section.hpp>
#include <vk/core/static_instance.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/drivers/icm20948.hpp>
#include <vk/drivers/ssd1306.hpp>
#include <vk/local_display/pages.hpp>
#include <vk/msg/imu_sample.hpp>

#include <array>
#include <cmath>
#include <cstdint>

namespace {

using example::Line;

// Down points opposite to the measured specific force, which is the upward reaction to gravity at rest.
const char* down_axis(const std::array<float, 3>& accel)
{
    const float abs_x = std::abs(accel[0]);
    const float abs_y = std::abs(accel[1]);
    const float abs_z = std::abs(accel[2]);

    if (abs_x >= abs_y && abs_x >= abs_z) {
        return accel[0] >= 0.0F ? "-X" : "+X";
    }
    if (abs_y >= abs_x && abs_y >= abs_z) {
        return accel[1] >= 0.0F ? "-Y" : "+Y";
    }
    return accel[2] >= 0.0F ? "-Z" : "+Z";
}

class ImuSubscriberItem final : public vk::core::WorkItem {
public:
    struct State {
        std::array<float, 3> accel{};
        std::uint32_t samples = 0;
    };

    explicit ImuSubscriberItem(vk::core::WorkQueue& queue)
        : WorkItem("imu_sub", queue)
    {
    }

    void start()
    {
        trigger_on(vk::core::topic<vk::msg::ImuSample>(0));
    }

    [[nodiscard]] State state() const
    {
        const vk::core::CriticalSection lock;
        return state_;
    }

protected:
    void run() override
    {
        vk::msg::ImuSample sample{};
        while (subscription_.copy(sample)) {
            const vk::core::CriticalSection lock;
            state_.accel = sample.accel;
            ++state_.samples;
        }
    }

private:
    vk::core::Subscription<vk::msg::ImuSample> subscription_{0};
    State state_{};
};

class DisplayItem final : public vk::core::WorkItem {
public:
    static constexpr std::uint32_t interval_us = 250'000;  // 4 Hz

    DisplayItem(const ImuSubscriberItem& subscriber, vk::drivers::Ssd1306& display, vk::core::WorkQueue& queue)
        : WorkItem("display", queue), subscriber_(subscriber), display_(display)
    {
    }

    void start()
    {
        schedule_periodic(interval_us);
    }

protected:
    void run() override
    {
        const auto now = queue().now_us();
        const auto state = subscriber_.state();

        if (start_us_ == 0) {
            start_us_ = now;
            last_rate_time_us_ = now;
            last_samples_ = state.samples;
        }

        // Measure sample rate: once a second, compute and report rate.
        const auto elapsed_rate_us = now - last_rate_time_us_;
        if (elapsed_rate_us >= 1'000'000) {
            const auto delta_samples = state.samples - last_samples_;
            rate_hz_ = static_cast<float>(delta_samples) * 1e6F / static_cast<float>(elapsed_rate_us);
            last_samples_ = state.samples;
            last_rate_time_us_ = now;
            example::report("rate", static_cast<std::uint32_t>(std::lround(rate_hz_)));
        } else if (rate_hz_ == 0.0F && now > start_us_) {
            // Initial rate estimate before the first full second has elapsed.
            rate_hz_ = static_cast<float>(state.samples) * 1e6F / static_cast<float>(now - start_us_);
        }

        const char* const down = down_axis(state.accel);

        // Report down axis and specific force at the display rate.
        example::report("down", down);
        example::report("ax", state.accel[0]);
        example::report("ay", state.accel[1]);
        example::report("az", state.accel[2]);

        const auto display_rate = static_cast<std::uint32_t>(std::lround(rate_hz_));

        screen_.clear();
        screen_.put(0, 0, "VEHKERNEL 06-IMU");
        screen_.put(2, 0, Line().add("AX: ").add_fixed(state.accel[0]).add(" M/S2").view());
        screen_.put(3, 0, Line().add("AY: ").add_fixed(state.accel[1]).add(" M/S2").view());
        screen_.put(4, 0, Line().add("AZ: ").add_fixed(state.accel[2]).add(" M/S2").view());
        screen_.put(5, 0, Line().add("RATE: ").add(display_rate).add(" HZ").view());
        screen_.put(6, 0, Line().add("DOWN: ").add(down).view());
        screen_.draw(display_.frame());

        static_cast<void>(display_.update(now, vk::drivers::MonoFrame::pages));
    }

private:
    const ImuSubscriberItem& subscriber_;
    vk::drivers::Ssd1306& display_;
    vk::local_display::TextScreen screen_{};
    std::uint64_t start_us_ = 0;
    std::uint64_t last_rate_time_us_ = 0;
    std::uint32_t last_samples_ = 0;
    float rate_hz_ = 0.0F;
};

vk::core::StaticInstance<ImuSubscriberItem> subscriber_item;
vk::core::StaticInstance<DisplayItem> display_item;

}  // namespace

int main(int argc, char** argv)
{
    example::init(argc, argv);

    auto& display = example::display();
    auto& imu = example::imu();
    auto& ctrl_queue = vk::core::work_queue(vk::core::WorkQueueTier::Ctrl);
    auto& slow_queue = vk::core::work_queue(vk::core::WorkQueueTier::Slow);

    subscriber_item.emplace(ctrl_queue);
    display_item.emplace(*subscriber_item, display, slow_queue);

    subscriber_item->start();
    display_item->start();
    imu.start();

    example::start_scheduler();

    return 0;
}
