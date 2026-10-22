// The user key's presses (ADR-0022), from samples taken every 10 ms.
//
// A level counts once three samples in a row agree (30 ms debounce). A key
// that reads pressed at boot is ignored until it has been released. A press
// released before 1 s is a short press. A press held for 1 s is a long
// press, reported once while held, then repeats every 0.2 s for as long as it
// stays held; the next press needs a release first, so a key held down or
// stuck gives at most one long press.
#pragma once

#include <cstdint>

namespace vk::local_display {

enum class KeyEvent : std::uint8_t {
    None,
    Short,   // released before the long press time
    Long,    // held for the long press time, once per press
    Repeat,  // still held, every repeat interval after the long press
};

class UserKey {
public:
    static constexpr std::uint32_t sample_interval_us = 10'000;
    static constexpr unsigned debounce_samples = 3;
    static constexpr std::uint64_t long_press_us = 1'000'000;
    static constexpr std::uint64_t repeat_interval_us = 200'000;

    // One sample: whether the key reads pressed, at `now_us`.
    KeyEvent sample(std::uint64_t now_us, bool pressed);

private:
    enum class Phase : std::uint8_t {
        Boot,      // nothing debounced yet
        Blocked,   // pressed at boot: waiting for a release
        Released,
        Pressed,   // pressed, before the long press
        Held,      // the long press reported, repeating
    };

    Phase phase_ = Phase::Boot;
    bool raw_ = false;
    unsigned agreeing_ = 0;  // samples in a row reading raw_
    std::uint64_t pressed_at_us_ = 0;
    std::uint64_t next_repeat_us_ = 0;
};

}  // namespace vk::local_display
