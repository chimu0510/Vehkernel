// Firmware entry point. For now the firmware only idles; the RTOS starts here later.
int main()
{
    for (;;) {
        __asm__ volatile("wfi");
    }
}
