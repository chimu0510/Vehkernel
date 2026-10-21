#include <vk/local_display/local_display.hpp>

#include <vk/hal/clock.hpp>

#include <cstddef>

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

LocalDisplay::LocalDisplay(drivers::Ssd1306& display, core::WorkQueue& queue)
    : core::WorkItem("local_display", queue), display_(display)
{
}

void LocalDisplay::start()
{
    schedule_periodic(interval_us);
}

void LocalDisplay::run()
{
    take(status_, inputs_.status);
    take(attitude_, inputs_.attitude);
    for (std::size_t instance = 0; instance < links_.size(); ++instance) {
        take(links_.at(instance), inputs_.links.at(instance));
    }
    render(page_.load(), inputs_, screen_);
    screen_.draw(display_.frame());
    display_.update(hal::SystemClock::now_us());
}

}  // namespace vk::local_display
