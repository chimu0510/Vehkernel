// SPDX-License-Identifier: GPL-3.0-or-later
// Example 05-bus: one work item publishes a counter that another, woken by it, draws on the OLED.
// ADR-0002 (bus backbone), ADR-0004 (FreeRTOS), ADR-0006 (schema), ADR-0008 (bus and work queues), ADR-0022 (local display), ADR-0025 (board examples).
// Lesson: examples/c50c-v1_1/05-bus/README.md
#include <line.hpp>
#include <support.hpp>
#include <vk/core/bus.hpp>
#include <vk/core/static_instance.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/drivers/ssd1306.hpp>
#include <vk/local_display/pages.hpp>
#include <vk/msg/example_counter.hpp>

#include <cstdint>

namespace {

using example::Line;

class PublisherItem final : public vk::core::WorkItem {
public:
    static constexpr std::uint32_t interval_us = 100'000;  // 10 Hz

    explicit PublisherItem(vk::core::WorkQueue& queue)
        : WorkItem("publisher", queue)
    {
    }

    void start()
    {
        schedule_periodic(interval_us);
    }

    [[nodiscard]] std::uint32_t count() const { return count_; }

protected:
    void run() override
    {
        ++count_;
        example::report("published", count_);

        vk::msg::ExampleCounter msg{};
        msg.timestamp = queue().now_us();
        msg.count = count_;
        publication_.publish(msg);
    }

private:
    vk::core::Publication<vk::msg::ExampleCounter> publication_{0};
    std::uint32_t count_ = 0;
};

class SubscriberItem final : public vk::core::WorkItem {
public:
    SubscriberItem(vk::drivers::Ssd1306& display, vk::core::WorkQueue& queue)
        : WorkItem("subscriber", queue), display_(display)
    {
    }

    void start()
    {
        trigger_on(vk::core::topic<vk::msg::ExampleCounter>(0));
    }

    [[nodiscard]] std::uint32_t latest_count() const { return latest_count_; }
    [[nodiscard]] std::uint32_t received_count() const { return received_count_; }

protected:
    void run() override
    {
        vk::msg::ExampleCounter sample{};
        while (subscription_.copy(sample)) {
            latest_count_ = sample.count;
            ++received_count_;
            example::report("received", sample.count);
        }

        screen_.clear();
        screen_.put(0, 0, "VEHKERNEL 05-BUS");
        screen_.put(2, 0, Line().add("COUNT: ").add(latest_count_).view());
        screen_.put(4, 0, Line().add("RECEIVED: ").add(received_count_).view());
        screen_.draw(display_.frame());

        static_cast<void>(display_.update(queue().now_us(), vk::drivers::MonoFrame::pages));
    }

private:
    vk::drivers::Ssd1306& display_;
    vk::core::Subscription<vk::msg::ExampleCounter> subscription_{0};
    std::uint32_t latest_count_ = 0;
    std::uint32_t received_count_ = 0;
    vk::local_display::TextScreen screen_{};
};

vk::core::StaticInstance<PublisherItem> publisher_item;
vk::core::StaticInstance<SubscriberItem> subscriber_item;

}  // namespace

int main(int argc, char** argv)
{
    example::init(argc, argv);

    auto& display = example::display();
    auto& io_queue = vk::core::work_queue(vk::core::WorkQueueTier::Io);
    auto& slow_queue = vk::core::work_queue(vk::core::WorkQueueTier::Slow);

    publisher_item.emplace(io_queue);
    subscriber_item.emplace(display, slow_queue);

    subscriber_item->start();
    publisher_item->start();

    example::start_scheduler();

    return 0;
}
