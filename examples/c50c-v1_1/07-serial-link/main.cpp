// SPDX-License-Identifier: GPL-3.0-or-later
// Example 08-serial-link: Custom protocol Link over SERIAL1 and SERIAL2 carrying IMU topics; OLED shows Link status.
// ADR-0002 (bus backbone and pluggable Links), ADR-0004 (FreeRTOS), ADR-0006 (schema and wire encoding), ADR-0008 (bus and work queues), ADR-0009 (layered HAL), ADR-0022 (local display), ADR-0025 (board examples).
// Lesson: examples/c50c-v1_1/08-serial-link/README.md
#include <line.hpp>
#include <support.hpp>
#include <vk/core/bus.hpp>
#include <vk/core/static_instance.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/drivers/icm20948.hpp>
#include <vk/drivers/ssd1306.hpp>
#include <vk/link/link.hpp>
#include <vk/link/topic_map.hpp>
#include <vk/local_display/pages.hpp>
#include <vk/msg/imu_sample.hpp>
#include <vk/msg/link_status.hpp>

#include <array>
#include <cstddef>
#include <cstdint>

namespace {

using example::Line;

constexpr vk::link::MaxRate status_rate{1.0F};

// Topic map: streams only what this example teaches (imu_sample) plus link_status for Link health.
struct ExampleStreams {
    explicit ExampleStreams(float imu_rate_hz) : imu(vk::link::every_sample, 0, imu_rate_hz) {}

    vk::link::TopicStreamOf<vk::msg::ImuSample> imu;
    std::array<vk::link::TopicStreamOf<vk::msg::LinkStatus>, vk::msg::LinkStatus::instances> links{
        vk::link::TopicStreamOf<vk::msg::LinkStatus>{status_rate, 0},
        vk::link::TopicStreamOf<vk::msg::LinkStatus>{status_rate, 1},
    };
    std::array<vk::link::TopicStream*, 1 + vk::msg::LinkStatus::instances> map{
        &imu, &links[0], &links[1],
    };
};

vk::link::TopicMap example_topic_map(float imu_rate_hz, std::size_t link)
{
    static std::array<vk::core::StaticInstance<ExampleStreams>, vk::msg::LinkStatus::instances> streams;
    auto& instance = streams.at(link);
    if (!instance.has_value()) {
        instance.emplace(imu_rate_hz);
    }
    return instance->map;
}

const char* state_name(vk::link::HandshakeState state)
{
    switch (state) {
    case vk::link::HandshakeState::Connected:
        return "CONNECTED";
    case vk::link::HandshakeState::PeerSilent:
        return "SILENT";
    case vk::link::HandshakeState::Waiting:
    default:
        return "WAITING";
    }
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
        schedule_periodic(interval_us);
    }

protected:
    void run() override
    {
        const auto now = queue().now_us();

        vk::msg::LinkStatus status{};
        while (status_sub_.copy(status)) {
            last_status_ = status;
            has_status_ = true;
        }

        const auto state = has_status_ ? static_cast<vk::link::HandshakeState>(last_status_.handshake_state)
                                        : vk::link::HandshakeState::Waiting;
        const auto tx = has_status_ ? last_status_.frames_sent : 0U;
        const auto rx = has_status_ ? last_status_.frames_received : 0U;
        const auto drops = has_status_ ? last_status_.drops : 0U;
        const auto scale = has_status_ ? last_status_.rate_scale : 100U;

        example::report("state", state_name(state));
        example::report("tx", tx);
        example::report("rx", rx);
        example::report("drops", drops);
        example::report("scale", scale);

        screen_.clear();
        screen_.put(0, 0, "VEHKERNEL 08-LINK");
        screen_.put(2, 0, Line().add("STATE: ").add(state_name(state)).view());
        screen_.put(3, 0, Line().add("TX FRAMES: ").add(tx).view());
        screen_.put(4, 0, Line().add("RX FRAMES: ").add(rx).view());
        screen_.put(5, 0, Line().add("DROPS: ").add(drops).view());
        screen_.put(6, 0, Line().add("SCALE: ").add(scale).add(" %").view());
        screen_.draw(display_.frame());

        static_cast<void>(display_.update(now, vk::drivers::MonoFrame::pages));
    }

private:
    vk::drivers::Ssd1306& display_;
    vk::local_display::TextScreen screen_{};
    vk::core::Subscription<vk::msg::LinkStatus> status_sub_{0};
    vk::msg::LinkStatus last_status_{};
    bool has_status_ = false;
};

vk::core::StaticInstance<DisplayItem> display_item;

}  // namespace

int main(int argc, char** argv)
{
    example::init(argc, argv);

    auto& display = example::display();
    auto& imu = example::imu();
    auto& slow_queue = vk::core::work_queue(vk::core::WorkQueueTier::Slow);

    display_item.emplace(display, slow_queue);

    example::start_links(example_topic_map);
    imu.start();
    display_item->start();

    example::start_scheduler();

    return 0;
}
