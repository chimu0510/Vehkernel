// The output watchdog (ADR-0005): every platform turns the actuator outputs
// off when actuator control has not refreshed them for a few control
// periods, so a stalled wq:ctrl cannot leave a motor running on its last
// duty. The platform refreshes it from each duty it is given and checks it
// from something that keeps running when the work queues do not, a timer
// interrupt on a board or a high-priority task in SITL.
#pragma once

#include <algorithm>
#include <atomic>
#include <cstdint>

namespace vk::hal {

class OutputWatchdog {
public:
    // Periods without a refresh before the outputs turn off.
    static constexpr std::uint32_t periods = 3;

    explicit OutputWatchdog(std::uint32_t period_us) : timeout_us_(periods * period_us) {}

    // Times are kept to 32 bits, which a Cortex-M4 reads and writes
    // atomically; differences wrap correctly for over an hour.
    void refresh(std::uint64_t now_us)
    {
        last_us_.store(std::max<std::uint32_t>(1, static_cast<std::uint32_t>(now_us)));
    }

    // Whether the outputs must be off now. Each stall counts as one trip.
    bool expired(std::uint64_t now_us)
    {
        const std::uint32_t last = last_us_.load();
        const bool stalled = last != 0 && static_cast<std::uint32_t>(now_us) - last > timeout_us_;
        if (stalled && last != tripped_at_) {
            tripped_at_ = last;
            trips_.fetch_add(1);
        }
        return stalled;
    }

    [[nodiscard]] std::uint32_t trips() const { return trips_.load(); }

private:
    std::uint32_t timeout_us_;
    std::atomic<std::uint32_t> last_us_{0};  // 0 until the first refresh
    std::uint32_t tripped_at_ = 0;           // the refresh the last trip followed
    std::atomic<std::uint32_t> trips_{0};
};

}  // namespace vk::hal
