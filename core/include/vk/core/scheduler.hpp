// Starting and stopping the RTOS scheduler (ADR-0004).
//
// Entry points start the scheduler through core/ so that only core/ and the
// platform layer call FreeRTOS directly.
#pragma once

namespace vk::core {

// Starts the scheduler. On hardware it never returns; in SITL it returns once
// stop_scheduler() has been called.
void start_scheduler();

// Stops the scheduler from a task (SITL only; the hardware port cannot stop).
void stop_scheduler();

}  // namespace vk::core
