// The local display as a work item on wq:slow (ADR-0022).
//
// Every 50 ms it renders the current page, or the menu while it is open, and
// lets the SSD1306 driver write every page of the screen that changed in the
// same run, so a key press is on the panel within one run and never half
// drawn. The vehicle_status, vehicle_attitude and link_status it shows are
// taken at most every 200 ms, so values change at most 5 Hz and stay
// readable. A run that changes nothing writes nothing; a full screen costs
// about 4.5 ms of busy-waiting on the C50C's GPIO lines, on the lowest-priority queue, so
// it never delays wq:ctrl or a Link. The menu runs on the same work queue,
// so it never changes during a run. It publishes nothing: producing modules
// never know it exists, and a display that is missing changes nothing.
#pragma once

#include <vk/core/bus.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/drivers/ssd1306.hpp>
#include <vk/local_display/local_menu.hpp>
#include <vk/local_display/pages.hpp>
#include <vk/msg/link_status.hpp>
#include <vk/msg/vehicle_attitude.hpp>
#include <vk/msg/vehicle_status.hpp>

#include <array>
#include <cstdint>

namespace vk::local_display {

class LocalDisplay final : public core::WorkItem {
public:
    static constexpr std::uint32_t interval_us = 50'000;        // how soon a key press shows
    static constexpr std::uint64_t topic_interval_us = 200'000;  // values change at most 5 Hz (ADR-0022)

    // `menu`, if any, must run on the same work queue.
    LocalDisplay(drivers::Ssd1306& display, const LocalMenu* menu, core::WorkQueue& queue);

    // Starts the periodic runs.
    void start();

protected:
    void run() override;

private:
    drivers::Ssd1306& display_;
    const LocalMenu* menu_;
    core::Subscription<msg::VehicleStatus> status_{0};
    core::Subscription<msg::VehicleAttitude> attitude_{0};
    std::array<core::Subscription<msg::LinkStatus>, msg::LinkStatus::instances> links_{
        core::Subscription<msg::LinkStatus>{0}, core::Subscription<msg::LinkStatus>{1}};
    Inputs inputs_;
    std::uint64_t next_topics_us_ = 0;
    TextScreen screen_;
};

}  // namespace vk::local_display
