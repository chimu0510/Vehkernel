// Firmware entry point. For now the firmware only idles; the RTOS starts here later.
#include <board.hpp>

static_assert(vk::hal::distinct_pins<vk::board::pins::all>,
              "the board generator must never assign one pin to two roles");

int main()
{
    for (;;) {
        __asm__ volatile("wfi");
    }
}
