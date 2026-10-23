// SPDX-License-Identifier: GPL-3.0-or-later
// Short critical sections for the bus (ADR-0008).
//
// Masks the interrupts that may call FreeRTOS and stops the scheduler from
// switching tasks until the guard goes out of scope. Keep the guarded code to
// a few copies; never block inside.
#pragma once

namespace vk::core {

class CriticalSection {
public:
    CriticalSection();
    ~CriticalSection();

    CriticalSection(const CriticalSection&) = delete;
    CriticalSection& operator=(const CriticalSection&) = delete;
    CriticalSection(CriticalSection&&) = delete;
    CriticalSection& operator=(CriticalSection&&) = delete;
};

// Stops the firmware on a broken invariant: halts on hardware, aborts in SITL.
[[noreturn]] void fail(const char* what);

}  // namespace vk::core
