// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/local_display/local_menu_task.hpp>

namespace vk::local_display {

LocalMenuTask::LocalMenuTask(const Config& config, core::Parameters& parameters,
                             std::span<const core::MenuParam> menu, core::WorkQueue& queue)
    : core::WorkItem("local_menu", queue), config_(config), menu_(parameters, menu)
{
}

void LocalMenuTask::start()
{
    schedule_periodic(UserKey::sample_interval_us);
}

void LocalMenuTask::run()
{
    const std::uint64_t now = queue().now_us();
    msg::ArmingState arming{};
    while (arming_.copy(arming)) {
        armed_ = arming.armed;
    }
    const bool pressed = config_.key_pressed != nullptr && config_.key_pressed();
    menu_.on_key(key_.sample(now, pressed), now, armed_);
    menu_.tick(now, armed_);

    msg::LocalMenu status{};
    status.state = static_cast<std::uint8_t>(menu_.state());
    status.item = menu_.item();
    status.last_result = menu_.last_result() ? static_cast<std::uint8_t>(*menu_.last_result()) : LocalMenu::no_result;
    status.last_parameter = menu_.last_result() ? menu_.last_parameter() : LocalMenu::no_item;
    status.reboot_required = menu_.reboot_required();
    const bool changed = !published_ || status.state != last_.state || status.item != last_.item ||
                         status.last_result != last_.last_result || status.last_parameter != last_.last_parameter ||
                         status.reboot_required != last_.reboot_required;
    if (changed || now >= next_status_us_) {
        status.timestamp = now;
        publication_.publish(status);
        last_ = status;
        published_ = true;
        next_status_us_ = now + status_interval_us;
    }
}

}  // namespace vk::local_display
