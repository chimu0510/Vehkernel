/*
 * Linker script for STM32F4 firmware. The board generator emits memory.ld
 * with the board's Flash layout and the MCU's RAM (ADR-0020); every region
 * size there is a budget, so outgrowing one fails the link.
 */
INCLUDE memory.ld

ENTRY(Reset_Handler)

/* Main stack, reserved in RAM so the RAM budget accounts for it. */
__vk_main_stack_size = 4K;
_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
    .boot_stub :
    {
        KEEP(*(.boot_vectors))
        KEEP(*(.boot_text))
    } > BOOT

    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } > FIRMWARE

    .text :
    {
        . = ALIGN(4);
        /* The app descriptor near the start of the image, as ArduPilot places
           it, so a bootloader searching for its signature finds it first
           (ADR-0023). */
        KEEP(*(.app_descriptor))
        *(.text .text.*)
        *(.rodata .rodata.*)
        KEEP(*(.init))
        KEEP(*(.fini))
        . = ALIGN(4);
    } > FIRMWARE

    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > FIRMWARE

    .preinit_array :
    {
        PROVIDE_HIDDEN(__preinit_array_start = .);
        KEEP(*(.preinit_array*))
        PROVIDE_HIDDEN(__preinit_array_end = .);
    } > FIRMWARE

    .init_array :
    {
        PROVIDE_HIDDEN(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array*))
        PROVIDE_HIDDEN(__init_array_end = .);
    } > FIRMWARE

    _sidata = LOADADDR(.data);

    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data .data.*)
        . = ALIGN(4);
        _edata = .;
    } > RAM AT > FIRMWARE

    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss .bss.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } > RAM

    .ccm (NOLOAD) :
    {
        *(.ccm .ccm.*)
    } > CCM

    /* The stack grows down from _estack; this only reserves its size after .bss. */
    .main_stack (NOLOAD) :
    {
        . = ALIGN(8);
        . += __vk_main_stack_size;
    } > RAM
}
