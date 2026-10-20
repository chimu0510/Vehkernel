/*
 * Minimal Cortex-M4F startup for STM32F407: vector table, memory init, FPU
 * enable, then main(). Exceptions and interrupts without a handler go to
 * Default_Handler; the named handlers below are weak, so the platform layer
 * and FreeRTOS override them by defining a function of the same name. Written
 * in C so the vector table is a constant initializer that stays in Flash.
 */
#include <stdint.h>

#define VK_SYSTEM_VECTORS 16
#define VK_STM32F407_IRQS 82

extern uint32_t _sidata[], _sdata[], _edata[], _sbss[], _ebss[], _estack[];
extern void (*__preinit_array_start[])(void);
extern void (*__preinit_array_end[])(void);
extern void (*__init_array_start[])(void);
extern void (*__init_array_end[])(void);

int main(void);
void Reset_Handler(void);
void Default_Handler(void);

#define VK_WEAK_HANDLER(name) void name(void) __attribute__((weak, alias("Default_Handler")))
VK_WEAK_HANDLER(SVC_Handler);     /* FreeRTOS */
VK_WEAK_HANDLER(PendSV_Handler);  /* FreeRTOS */
VK_WEAK_HANDLER(SysTick_Handler); /* FreeRTOS tick */
VK_WEAK_HANDLER(TIM6_DAC_IRQHandler); /* output watchdog */
VK_WEAK_HANDLER(TIM7_IRQHandler); /* microsecond clock overflow */

/* Vector numbers: system exceptions first, then IRQn + VK_SYSTEM_VECTORS. */
#define VK_SVCALL_VECTOR  11
#define VK_PENDSV_VECTOR  14
#define VK_SYSTICK_VECTOR 15
#define VK_TIM6_VECTOR    (VK_SYSTEM_VECTORS + 54)
#define VK_TIM7_VECTOR    (VK_SYSTEM_VECTORS + 55)
#define VK_LAST_VECTOR    (VK_SYSTEM_VECTORS + VK_STM32F407_IRQS - 1)

typedef void (*vk_vector_t)(void);

__attribute__((section(".isr_vector"), used))
const vk_vector_t vk_vector_table[VK_SYSTEM_VECTORS + VK_STM32F407_IRQS] = {
    [0] = (vk_vector_t)_estack,
    [1] = Reset_Handler,
    [2 ... VK_SVCALL_VECTOR - 1] = Default_Handler,
    [VK_SVCALL_VECTOR] = SVC_Handler,
    [VK_SVCALL_VECTOR + 1 ... VK_PENDSV_VECTOR - 1] = Default_Handler,
    [VK_PENDSV_VECTOR] = PendSV_Handler,
    [VK_SYSTICK_VECTOR] = SysTick_Handler,
    [VK_SYSTICK_VECTOR + 1 ... VK_TIM6_VECTOR - 1] = Default_Handler,
    [VK_TIM6_VECTOR] = TIM6_DAC_IRQHandler,
    [VK_TIM7_VECTOR] = TIM7_IRQHandler,
    [VK_TIM7_VECTOR + 1 ... VK_LAST_VECTOR] = Default_Handler,
};

void Default_Handler(void)
{
    for (;;) {
    }
}

void Reset_Handler(void)
{
    /*
     * Keep the debug clocks running in Sleep, Stop and Standby (DBGMCU_CR
     * DBG_SLEEP, DBG_STOP, DBG_STANDBY). Without this, HCLK stops during WFI
     * and a debugger can no longer attach (RM0090, DBGMCU_CR). The cost is a
     * slightly higher sleep current.
     */
    volatile uint32_t *const dbgmcu_cr = (volatile uint32_t *)0xE0042004u;
    *dbgmcu_cr |= 0x7u;

    /* Point VTOR at this table even when started without the boot stub, e.g. by a debugger. */
    volatile uint32_t *const vtor = (volatile uint32_t *)0xE000ED08u;
    *vtor = (uint32_t)(uintptr_t)vk_vector_table;

    for (uint32_t *src = _sidata, *dst = _sdata; dst < _edata;) {
        *dst++ = *src++;
    }
    for (uint32_t *dst = _sbss; dst < _ebss;) {
        *dst++ = 0;
    }

    /* Grant full access to CP10 and CP11 (the FPU) before any float code runs. */
    volatile uint32_t *const cpacr = (volatile uint32_t *)0xE000ED88u;
    *cpacr |= 0xFu << 20;
    __asm__ volatile("dsb\n\tisb" ::: "memory");

    for (void (**fn)(void) = __preinit_array_start; fn < __preinit_array_end; ++fn) {
        (*fn)();
    }
    for (void (**fn)(void) = __init_array_start; fn < __init_array_end; ++fn) {
        (*fn)();
    }

    main();
    Default_Handler();
}
