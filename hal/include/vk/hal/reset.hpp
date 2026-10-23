// Restarting the board (ADR-0023). Every platform provides a reset that
// starts the firmware again or stops in the bootloader, leaving the request
// where the board's bootloader reads it: on the STM32 an RTC backup register
// with the ArduPilot or PX4 value; in SITL an exit status. It never returns.
#pragma once

#include <cstdint>

namespace vk::hal {

enum class BootTarget : std::uint8_t {
    Firmware,    // start the firmware again
    Bootloader,  // stay in the bootloader until flashed or reset
};

// A platform's reset.
using Reset = void (*)(BootTarget target);

}  // namespace vk::hal
