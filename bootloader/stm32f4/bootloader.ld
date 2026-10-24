/* SPDX-License-Identifier: GPL-3.0-or-later */
/*
 * Linker script for the Vehkernel bootloader on STM32F4 (ADR-0023). The board
 * generator emits memory.ld with the board's Flash layout (ADR-0020): the
 * bootloader's code and constants go in BOOT, whose size is its budget, so
 * outgrowing it fails the link (ADR-0017). It uses RAM only, leaving CCM to
 * the firmware.
 */
INCLUDE memory.ld

ENTRY(vk_bootloader_reset)

__vk_bootloader_stack_size = 2K;
_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
    .isr_vector :
    {
        KEEP(*(.isr_vector))
    } > BOOT

    .text :
    {
        . = ALIGN(4);
        *(.text .text.*)
        *(.rodata .rodata.*)
        . = ALIGN(4);
    } > BOOT

    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > BOOT

    _sidata = LOADADDR(.data);

    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data .data.*)
        . = ALIGN(4);
        _edata = .;
    } > RAM AT > BOOT

    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss .bss.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > RAM

    /* The stack grows down from _estack; this only reserves its size after .bss. */
    .main_stack (NOLOAD) :
    {
        . = ALIGN(8);
        . += __vk_bootloader_stack_size;
    } > RAM
}
