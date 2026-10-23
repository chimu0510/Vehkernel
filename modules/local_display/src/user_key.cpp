// SPDX-License-Identifier: GPL-3.0-or-later
#include <vk/local_display/user_key.hpp>

namespace vk::local_display {

KeyEvent UserKey::sample(std::uint64_t now_us, bool pressed)
{
    if (pressed == raw_) {
        if (agreeing_ < debounce_samples) {
            ++agreeing_;
        }
    } else {
        raw_ = pressed;
        agreeing_ = 1;
    }
    if (agreeing_ < debounce_samples) {
        // Not settled: a pending long press or repeat still runs on time.
        pressed = phase_ == Phase::Pressed || phase_ == Phase::Held;
    }

    switch (phase_) {
    case Phase::Boot:
        if (agreeing_ >= debounce_samples) {
            phase_ = pressed ? Phase::Blocked : Phase::Released;
        }
        return KeyEvent::None;
    case Phase::Blocked:
        if (!pressed) {
            phase_ = Phase::Released;
        }
        return KeyEvent::None;
    case Phase::Released:
        if (pressed) {
            phase_ = Phase::Pressed;
            pressed_at_us_ = now_us;
        }
        return KeyEvent::None;
    case Phase::Pressed:
        if (!pressed) {
            phase_ = Phase::Released;
            return KeyEvent::Short;
        }
        if (now_us - pressed_at_us_ >= long_press_us) {
            phase_ = Phase::Held;
            next_repeat_us_ = now_us + repeat_interval_us;
            return KeyEvent::Long;
        }
        return KeyEvent::None;
    case Phase::Held:
        if (!pressed) {
            phase_ = Phase::Released;
            return KeyEvent::None;
        }
        if (now_us >= next_repeat_us_) {
            next_repeat_us_ += repeat_interval_us;
            return KeyEvent::Repeat;
        }
        return KeyEvent::None;
    }
    return KeyEvent::None;
}

}  // namespace vk::local_display
