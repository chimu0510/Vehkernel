// SPDX-License-Identifier: GPL-3.0-or-later
// Example 04-work-queues: two work items at different rates counting on the OLED.
// ADR-0004 (FreeRTOS), ADR-0008 (bus and work queues), ADR-0022 (local display), ADR-0025 (board examples).
// Lesson: examples/c50c-v1_1/04-work-queues/README.md
#include <line.hpp>
#include <support.hpp>
#include <vk/core/static_instance.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/drivers/ssd1306.hpp>
#include <vk/local_display/pages.hpp>

#include <cstdint>

namespace {

using example::Line;

class SlowItem final : public vk::core::WorkItem {
public:
    static constexpr std::uint32_t interval_us = 500'000;  // 2 Hz

    explicit SlowItem(vk::core::WorkQueue& queue)
        : WorkItem("slow", queue)
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
        example::report("slow", count_);
    }

private:
    std::uint32_t count_ = 0;
};

class FastItem final : public vk::core::WorkItem {
public:
    static constexpr std::uint32_t interval_us = 100'000;  // 10 Hz

    FastItem(const SlowItem& slow_item, vk::drivers::Ssd1306& display, vk::core::WorkQueue& queue)
        : WorkItem("fast", queue), slow_item_(slow_item), display_(display)
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
        example::report("fast", count_);

        screen_.clear();
        screen_.put(0, 0, "VEHKERNEL 04-WQ");
        screen_.put(2, 0, Line().add("FAST: ").add(count_).view());
        screen_.put(4, 0, Line().add("SLOW: ").add(slow_item_.count()).view());
        screen_.draw(display_.frame());

        static_cast<void>(display_.update(queue().now_us(), vk::drivers::MonoFrame::pages));
    }

private:
    const SlowItem& slow_item_;
    vk::drivers::Ssd1306& display_;
    std::uint32_t count_ = 0;
    vk::local_display::TextScreen screen_{};
};

vk::core::StaticInstance<SlowItem> slow_item;
vk::core::StaticInstance<FastItem> fast_item;

}  // namespace

int main(int argc, char** argv)
{
    example::init(argc, argv);

    auto& display = example::display();
    auto& queue = vk::core::work_queue(vk::core::WorkQueueTier::Slow);

    slow_item.emplace(queue);
    fast_item.emplace(*slow_item, display, queue);

    slow_item->start();
    fast_item->start();

    example::start_scheduler();

    return 0;
}
