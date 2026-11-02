// SPDX-License-Identifier: GPL-3.0-or-later
// Example 08-magnetometer: AK09916 magnetometer read through ICM20948 FIFO; OLED shows magnetic field, magnitude and heading; the Link carries the samples.
// ADR-0002 (bus backbone and pluggable Links), ADR-0004 (FreeRTOS), ADR-0006 (schema), ADR-0008 (bus and work queues), ADR-0009 (layered HAL), ADR-0014 (estimation architecture), ADR-0015 (selectable timing), ADR-0022 (local display), ADR-0025 (board examples).
// Lesson: examples/c50c-v1_1/08-magnetometer/README.md
#include <line.hpp>
#include <support.hpp>
#include <vk/core/bus.hpp>
#include <vk/core/critical_section.hpp>
#include <vk/core/static_instance.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/drivers/icm20948.hpp>
#include <vk/drivers/ssd1306.hpp>
#include <vk/link/link.hpp>
#include <vk/link/topic_map.hpp>
#include <vk/local_display/pages.hpp>
#include <vk/msg/link_status.hpp>
#include <vk/msg/magnetometer_sample.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>

namespace {

using example::Line;

constexpr vk::link::MaxRate status_rate{1.0F};

// Topic map: every magnetometer sample, for link_monitor.py mag-read, plus link_status for Link health.
struct ExampleStreams {
    vk::link::TopicStreamOf<vk::msg::MagnetometerSample> mag{vk::link::every_sample, 0,
                                                             vk::msg::MagnetometerSample::nominal_rate_hz};
    std::array<vk::link::TopicStreamOf<vk::msg::LinkStatus>, vk::msg::LinkStatus::instances> links{
        vk::link::TopicStreamOf<vk::msg::LinkStatus>{status_rate, 0},
        vk::link::TopicStreamOf<vk::msg::LinkStatus>{status_rate, 1},
    };
    std::array<vk::link::TopicStream*, 1 + vk::msg::LinkStatus::instances> map{
        &mag, &links[0], &links[1],
    };
};

vk::link::TopicMap example_topic_map(float /*imu_rate_hz*/, std::size_t link)
{
    static std::array<vk::core::StaticInstance<ExampleStreams>, vk::msg::LinkStatus::instances> streams;
    auto& instance = streams.at(link);
    if (!instance.has_value()) {
        instance.emplace();
    }
    return instance->map;
}

class MagSubscriberItem final : public vk::core::WorkItem {
public:
    struct State {
        std::array<float, 3> field{};
        std::uint64_t timestamp_us = 0;  // when the latest sample was read
        std::uint32_t samples = 0;
    };

    explicit MagSubscriberItem(vk::core::WorkQueue& queue)
        : WorkItem("mag_sub", queue)
    {
    }

    void start()
    {
        trigger_on(vk::core::topic<vk::msg::MagnetometerSample>(0));
    }

    [[nodiscard]] State state() const
    {
        const vk::core::CriticalSection lock;
        return state_;
    }

protected:
    void run() override
    {
        vk::msg::MagnetometerSample sample{};
        while (subscription_.copy(sample)) {
            const vk::core::CriticalSection lock;
            state_.field = sample.field;
            state_.timestamp_us = sample.timestamp;
            ++state_.samples;
        }
    }

private:
    vk::core::Subscription<vk::msg::MagnetometerSample> subscription_{0};
    State state_{};
};

class DisplayItem final : public vk::core::WorkItem {
public:
    static constexpr std::uint32_t interval_us = 250'000;  // 4 Hz

    DisplayItem(const MagSubscriberItem& subscriber, vk::drivers::Ssd1306& display, vk::core::WorkQueue& queue)
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

        const float bx = state.field[0];
        const float by = state.field[1];
        const float bz = state.field[2];
        const float norm = std::sqrt(bx * bx + by * by + bz * bz);

        // Heading of magnetic north for a level board: atan2(-y, x) in the body frame
        // (x forward, y right, z down).
        const float heading_rad = std::atan2(-by, bx);
        const float heading_deg = heading_rad * (180.0F / std::numbers::pi_v<float>);

        // Report heading and field components at the display rate, with the time of
        // the sample they come from.
        example::report("heading_us", state.timestamp_us);
        example::report("heading", heading_deg);
        example::report("bx", bx);
        example::report("by", by);
        example::report("bz", bz);
        example::report("norm", norm);

        const auto display_rate = static_cast<std::uint32_t>(std::lround(rate_hz_));

        screen_.clear();
        screen_.put(0, 0, "VEHKERNEL 08-MAG");
        screen_.put(2, 0, Line().add("BX: ").add_fixed(bx).add(" UT").view());
        screen_.put(3, 0, Line().add("BY: ").add_fixed(by).add(" UT").view());
        screen_.put(4, 0, Line().add("BZ: ").add_fixed(bz).add(" UT").view());
        screen_.put(5, 0, Line().add("NORM: ").add_fixed(norm).add(" UT").view());
        screen_.put(6, 0, Line().add("RATE: ").add(display_rate).add(" HZ").view());
        screen_.put(7, 0, Line().add("HEADING: ").add_fixed(heading_deg).add(" DEG").view());
        screen_.draw(display_.frame());

        static_cast<void>(display_.update(now, vk::drivers::MonoFrame::pages));
    }

private:
    const MagSubscriberItem& subscriber_;
    vk::drivers::Ssd1306& display_;
    vk::local_display::TextScreen screen_{};
    std::uint64_t start_us_ = 0;
    std::uint64_t last_rate_time_us_ = 0;
    std::uint32_t last_samples_ = 0;
    float rate_hz_ = 0.0F;
};

vk::core::StaticInstance<MagSubscriberItem> subscriber_item;
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

    example::start_links(example_topic_map);
    subscriber_item->start();
    display_item->start();
    imu.start();

    example::start_scheduler();

    return 0;
}
