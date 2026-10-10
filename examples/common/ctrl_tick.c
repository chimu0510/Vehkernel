#include "ctrl_tick.h"

#include "board.h"

#define TIMER_HZ     1000000U
#define IRQ_PRIORITY 5U

static TIM_HandleTypeDef s_tim;
static volatile ctrl_tick_fn s_fn;
static volatile uint32_t s_last_us, s_max_us, s_overruns;

void ctrl_tick_start(uint32_t hz, ctrl_tick_fn fn)
{
    __HAL_RCC_TIM6_CLK_ENABLE();
    s_fn = fn;
    s_last_us = s_max_us = s_overruns = 0;

    /* APB1 預除 4，計時器時脈為 PCLK1 × 2 = 84 MHz。 */
    s_tim.Instance = TIM6;
    s_tim.Init.Prescaler = 2U * HAL_RCC_GetPCLK1Freq() / TIMER_HZ - 1U;
    s_tim.Init.CounterMode = TIM_COUNTERMODE_UP;
    s_tim.Init.Period = TIMER_HZ / hz - 1U;
    s_tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&s_tim);

    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, IRQ_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    HAL_TIM_Base_Start_IT(&s_tim);
}

void ctrl_tick_stop(void)
{
    HAL_TIM_Base_Stop_IT(&s_tim);
    HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
}

uint32_t ctrl_tick_last_us(void)
{
    return s_last_us;
}

uint32_t ctrl_tick_max_us(void)
{
    return s_max_us;
}

uint32_t ctrl_tick_overruns(void)
{
    return s_overruns;
}

/* 不經過 HAL_TIM_IRQHandler，直接清旗標，減少每次中斷的負擔。 */
void TIM6_DAC_IRQHandler(void)
{
    if (!(TIM6->SR & TIM_SR_UIF)) {
        return;
    }
    TIM6->SR = ~TIM_SR_UIF;

    const uint32_t t0 = DWT->CYCCNT;
    if (s_fn != NULL) {
        s_fn();
    }
    const uint32_t us = (DWT->CYCCNT - t0) / (SystemCoreClock / 1000000U);
    s_last_us = us;
    if (us > s_max_us) {
        s_max_us = us;
    }
    /* 回呼期間計時器已經再次溢位，代表漏掉了一個週期。 */
    if (TIM6->SR & TIM_SR_UIF) {
        s_overruns++;
    }
}
