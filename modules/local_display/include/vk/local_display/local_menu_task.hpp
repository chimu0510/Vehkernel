// The user key and the local menu as a work item on wq:slow (ADR-0022).
//
// Every 10 ms it samples the user key, turns its presses into menu actions
// with the arming state from arming_state, and publishes local_menu on every
// change and once a second, which the commander reads to refuse arming
// while the menu is open. It writes parameters only through the parameter
// service and has no other output: no setpoints, mode, arming or emergency
// stop requests. The local display shows the menu, on the same work queue.
#pragma once

#include <vk/core/bus.hpp>
#include <vk/core/parameters.hpp>
#include <vk/core/work_queue.hpp>
#include <vk/local_display/local_menu.hpp>
#include <vk/local_display/user_key.hpp>
#include <vk/msg/arming_state.hpp>
#include <vk/msg/local_menu.hpp>

#include <cstdint>
#include <span>

namespace vk::local_display {

class LocalMenuTask final : public core::WorkItem {
public:
    static constexpr std::uint32_t status_interval_us = 1'000'000;

    struct Config {
        bool (*key_pressed)() = nullptr;  // reads the user key, true while pressed
    };

    LocalMenuTask(const Config& config, core::Parameters& parameters, std::span<const core::MenuParam> menu,
                  core::WorkQueue& queue);

    // Starts the periodic runs.
    void start();

    [[nodiscard]] const LocalMenu& menu() const { return menu_; }

protected:
    void run() override;

private:
    Config config_;
    UserKey key_;
    LocalMenu menu_;
    core::Subscription<msg::ArmingState> arming_{0};
    bool armed_ = false;
    core::Publication<msg::LocalMenu> publication_;
    msg::LocalMenu last_{};
    bool published_ = false;
    std::uint64_t next_status_us_ = 0;
};

}  // namespace vk::local_display
