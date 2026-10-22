// The local display as a work item on wq:slow (ADR-0022).
//
// Every 200 ms it takes the latest vehicle_status, vehicle_attitude and
// link_status, renders the current page and lets the SSD1306 driver write the
// pages of the screen that changed, at most two per run, so a display that
// busy-waits on GPIO lines never delays wq:ctrl or a Link. With a local menu
// it shows the menu's page, or the menu itself while it is open; the menu
// runs on the same work queue, so it never changes during a run. It publishes
// nothing: producing modules never know it exists, and a display that is
// missing changes nothing.
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
    static constexpr std::uint32_t interval_us = 200'000;  // at most 5 Hz (ADR-0022)

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
    TextScreen screen_;
};

}  // namespace vk::local_display
