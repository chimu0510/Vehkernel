/* SPDX-License-Identifier: GPL-3.0-or-later */
/*
 * The Vehkernel bootloader's vector table, at the start of the BOOT region:
 * the stack, reset, the faults and SysTick, the only interrupt it enables.
 * Written in C so the table is a constant initializer that stays in Flash.
 */
#include <stdint.h>

extern uint32_t _estack[];

void vk_bootloader_reset(void);
void vk_bootloader_fault(void);
void SysTick_Handler(void);

typedef void (*vk_vector_t)(void);

__attribute__((section(".isr_vector"), used))
const vk_vector_t vk_bootloader_vectors[16] = {
    [0] = (vk_vector_t)_estack,
    [1] = vk_bootloader_reset,
    [2 ... 6] = vk_bootloader_fault, /* NMI, HardFault, MemManage, BusFault, UsageFault */
    [11 ... 12] = vk_bootloader_fault, /* SVCall, DebugMonitor */
    [14] = vk_bootloader_fault, /* PendSV */
    [15] = SysTick_Handler,
};
