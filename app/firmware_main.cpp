// Firmware entry point. For now the firmware only idles; the RTOS starts here later.
#include <board.hpp>
#include <vk/config.h>

#ifndef CONFIG_PLATFORM_STM32F4
#error "the Kconfig layers must select the STM32F4 platform for firmware builds"
#endif

static_assert(vk::hal::distinct_pins<vk::board::pins::all>,
              "the board generator must never assign one pin to two roles");

#ifdef VK_FILL_FLASH_KB
// Size budget smoke test only: an image too large for the FIRMWARE region.
const unsigned char vk_flash_fill[VK_FILL_FLASH_KB * 1024] = {1};  // NOLINT
#endif

int main()
{
#ifdef VK_FILL_FLASH_KB
    // Referenced so --gc-sections keeps the fill.
    __asm__ volatile("" : : "r"(vk_flash_fill));
#endif
    for (;;) {
        __asm__ volatile("wfi");
    }
}
