// SITL entry point. The FreeRTOS POSIX port starts here later (ADR-0018).
#include <vk/config.h>

#ifndef CONFIG_PLATFORM_SITL
#error "the Kconfig layers must select the SITL platform for SITL builds"
#endif

int main()
{
    return 0;
}
