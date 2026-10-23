// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Jump stub for the BOOT region (ADR-0020). Until a bootloader exists, S0
 * holds only this: a two-entry vector table whose reset handler points VTOR at
 * the firmware's vector table, loads its initial stack pointer and jumps to
 * its reset handler. Everything lives in .boot_* sections the linker script
 * places in BOOT.
 */
#include <stdint.h>

extern uint32_t __vk_flash_firmware_start[];
extern uint32_t _estack[];

void vk_boot_stub_reset(void);

__attribute__((section(".boot_vectors"), used))
const uintptr_t vk_boot_vectors[2] = {
    (uintptr_t)_estack,
    (uintptr_t)vk_boot_stub_reset,
};

__attribute__((section(".boot_text"), naked, noreturn, used))
void vk_boot_stub_reset(void)
{
    __asm__ volatile(
        "ldr r0, =__vk_flash_firmware_start\n\t"
        "ldr r1, =0xE000ED08\n\t" /* SCB->VTOR */
        "str r0, [r1]\n\t"
        "dsb\n\t"
        "isb\n\t"
        "ldr r1, [r0]\n\t"
        "msr msp, r1\n\t"
        "ldr r1, [r0, #4]\n\t"
        "bx r1\n\t"
        ".ltorg");
}
