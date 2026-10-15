#include "stm32f4xx_hal.h"

#include "board.h"

void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* 任何 fault 都停在這裡並讓紅燈快閃，方便在沒有除錯器時辨識。 */
static void fault_blink(void)
{
    __disable_irq();
    for (;;) {
        led_set(true, false, false);
        for (volatile uint32_t i = 0; i < 2000000U; i++) {
        }
        led_set(false, false, false);
        for (volatile uint32_t i = 0; i < 2000000U; i++) {
        }
    }
}

void HardFault_Handler(void)  { fault_blink(); }
void MemManage_Handler(void)  { fault_blink(); }
void BusFault_Handler(void)   { fault_blink(); }
void UsageFault_Handler(void) { fault_blink(); }
