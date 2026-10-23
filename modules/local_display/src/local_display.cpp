// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/local_display/local_display.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>

namespace vk::local_display {

namespace {

// Keeps the newest message of a subscription, if one arrived.
template <class T>
void take(core::Subscription<T>& subscription, std::optional<T>& latest)
{
    T message{};
    while (subscription.copy(message)) {
        latest = message;
    }
}

}  // namespace

LocalDisplay::LocalDisplay(drivers::Ssd1306& display, const LocalMenu* menu, core::WorkQueue& queue)
    : core::WorkItem("local_display", queue), display_(display), menu_(menu)
{
}

void LocalDisplay::start()
{
    schedule_periodic(interval_us);
}

void LocalDisplay::run()
{
    const std::uint64_t now = queue().now_us();
    if (now >= next_topics_us_) {
        take(status_, inputs_.status);
        take(attitude_, inputs_.attitude);
        for (std::size_t instance = 0; instance < links_.size(); ++instance) {
            take(links_.at(instance), inputs_.links.at(instance));
        }
        // Keeps the 200 ms rhythm, unless the queue fell a whole interval behind.
        next_topics_us_ += topic_interval_us;
        if (next_topics_us_ <= now) {
            next_topics_us_ = now + topic_interval_us;
        }
    }
    if (menu_ != nullptr && menu_->state() != MenuState::Closed) {
        render_menu(*menu_, screen_);
    } else {
        render(menu_ != nullptr ? menu_->page() : Page::Status, inputs_, screen_);
    }
    screen_.draw(display_.frame());
    display_.update(now, drivers::MonoFrame::pages);
}

}  // namespace vk::local_display
