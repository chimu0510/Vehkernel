// SPDX-License-Identifier: GPL-3.0-or-later
// Example 09-parameters: Parameter system and storage in non-volatile Flash (PARAMS);
// value set over Link shown on OLED and kept across reset.
// ADR-0002 (bus backbone and pluggable Links), ADR-0004 (FreeRTOS), ADR-0006 (schema and wire encoding),
// ADR-0008 (bus and work queues), ADR-0009 (layered HAL), ADR-0012 (parameter system),
// ADR-0020 (internal Flash layout and write policy), ADR-0022 (local display), ADR-0025 (board examples).
// Lesson: examples/c50c-v1_1/09-parameters/README.md
#include <line.hpp>
#include <support.hpp>
#include <vk/core/bus.hpp>
#include <vk/core/static_instance.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/drivers/ssd1306.hpp>
#include <vk/link/link.hpp>
#include <vk/link/topic_map.hpp>
#include <vk/local_display/pages.hpp>
#include <vk/msg/link_status.hpp>
#include <vk/msg/parameter_status.hpp>
#include <vk/msg/parameter_update.hpp>
#include <vk/params/params.hpp>

#include <array>
#include <cstddef>
#include <cstdint>

namespace {

using example::Line;

constexpr vk::link::MaxRate status_rate{1.0F};

// Topic map: streams parameter_status (storage health and stats), parameter_update (on change),
// and link_status for Link health.
struct ExampleStreams {
    vk::link::TopicStreamOf<vk::msg::ParameterStatus> param_status{status_rate, 0};
    vk::link::TopicStreamOf<vk::msg::ParameterUpdate> param_update{vk::link::every_sample, 0, 1.0F};
    std::array<vk::link::TopicStreamOf<vk::msg::LinkStatus>, vk::msg::LinkStatus::instances> links{
        vk::link::TopicStreamOf<vk::msg::LinkStatus>{status_rate, 0},
        vk::link::TopicStreamOf<vk::msg::LinkStatus>{status_rate, 1},
    };
    std::array<vk::link::TopicStream*, 2 + vk::msg::LinkStatus::instances> map{
        &param_status, &param_update, &links[0], &links[1],
    };
};

vk::link::TopicMap example_topic_map(float /*unused*/, std::size_t link)
{
    static std::array<vk::core::StaticInstance<ExampleStreams>, vk::msg::LinkStatus::instances> streams;
    auto& instance = streams.at(link);
    if (!instance.has_value()) {
        instance.emplace();
    }
    return instance->map;
}

class DisplayItem final : public vk::core::WorkItem {
public:
    static constexpr std::uint32_t interval_us = 250'000;  // 4 Hz

    DisplayItem(vk::drivers::Ssd1306& display, vk::core::WorkQueue& queue)
        : WorkItem("display", queue), display_(display)
    {
    }

    void start()
    {
        trigger_on(vk::core::topic<vk::msg::ParameterUpdate>(0));
        schedule_periodic(interval_us);
    }

protected:
    void run() override
    {
        const auto now = queue().now_us();

        vk::msg::ParameterUpdate update{};
        while (update_sub_.copy(update)) {
            // Re-read directly from RAM struct when parameter_update is published.
            param_value_ = vk::params::att.kp;
        }

        vk::msg::ParameterStatus status{};
        while (status_sub_.copy(status)) {
            last_status_ = status;
            has_status_ = true;
        }

        const bool saved = !has_status_ || last_status_.unsaved == 0;
        const auto bytes_used = has_status_ ? last_status_.bytes_used : 8U;
        const auto sector = has_status_ ? last_status_.active_sector : 0U;

        example::report("param", "ATT_KP");
        example::report("value", param_value_);
        example::report("saved", saved ? 1 : 0);
        example::report("bytes_used", bytes_used);
        example::report("sector", static_cast<std::uint32_t>(sector));

        screen_.clear();
        screen_.put(0, 0, "VEHKERNEL 09-PARAMS");
        screen_.put(2, 0, "PARAM: ATT_KP");
        screen_.put(3, 0, Line().add("VALUE: ").add_float(param_value_).view());
        screen_.put(4, 0, Line().add("SAVED: ").add(saved ? "YES" : "NO").view());
        screen_.put(5, 0, Line().add("BYTES USED: ").add(bytes_used).view());
        screen_.put(6, 0, Line().add("SECTOR: ").add(static_cast<std::uint32_t>(sector)).view());
        screen_.draw(display_.frame());

        static_cast<void>(display_.update(now, vk::drivers::MonoFrame::pages));
    }

private:
    vk::drivers::Ssd1306& display_;
    vk::local_display::TextScreen screen_{};
    vk::core::Subscription<vk::msg::ParameterUpdate> update_sub_{0};
    vk::core::Subscription<vk::msg::ParameterStatus> status_sub_{0};
    float param_value_ = vk::params::att.kp;
    vk::msg::ParameterStatus last_status_{};
    bool has_status_ = false;
};

vk::core::StaticInstance<DisplayItem> display_item;

}  // namespace

int main(int argc, char** argv)
{
    example::init(argc, argv);
    example::start_parameters();

    auto& display = example::display();
    auto& slow_queue = vk::core::work_queue(vk::core::WorkQueueTier::Slow);

    display_item.emplace(display, slow_queue);

    example::start_links(example_topic_map);
    display_item->start();

    example::start_scheduler();

    return 0;
}
