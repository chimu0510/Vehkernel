#include "encoder.h"

#include <string.h>

#include "board.h"

typedef struct {
    TIM_TypeDef *tim;
    GPIO_TypeDef *port1;
    uint16_t pin1;
    GPIO_TypeDef *port2;
    uint16_t pin2;
    uint8_t af;
} encoder_hw_t;

static const encoder_hw_t k_hw[ENC_COUNT] = {
    {ENC_A_TIM, ENC_A1_PORT, ENC_A1_PIN, ENC_A2_PORT, ENC_A2_PIN, ENC_A_AF},
    {ENC_B_TIM, ENC_B1_PORT, ENC_B1_PIN, ENC_B2_PORT, ENC_B2_PIN, ENC_B_AF},
    {ENC_C_TIM, ENC_C1_PORT, ENC_C1_PIN, ENC_C2_PORT, ENC_C2_PIN, ENC_C_AF},
    {ENC_D_TIM, ENC_D1_PORT, ENC_D1_PIN, ENC_D2_PORT, ENC_D2_PIN, ENC_D_AF},
};

static TIM_HandleTypeDef s_tim[ENC_COUNT];
static uint16_t s_last[ENC_COUNT];
static int32_t s_total[ENC_COUNT];

static void init_one(encoder_id_t id)
{
    const encoder_hw_t *hw = &k_hw[id];
    GPIO_InitTypeDef io = {0};
    TIM_Encoder_InitTypeDef enc = {0};

    io.Mode = GPIO_MODE_AF_PP;
    io.Pull = GPIO_PULLUP;
    io.Speed = GPIO_SPEED_FREQ_LOW;
    io.Alternate = hw->af;
    io.Pin = hw->pin1;
    HAL_GPIO_Init(hw->port1, &io);
    io.Pin = hw->pin2;
    HAL_GPIO_Init(hw->port2, &io);

    /* 32 位元的 TIM2/TIM5 也只用低 16 位元，讓四路的處理方式一致。 */
    s_tim[id].Instance = hw->tim;
    s_tim[id].Init.Prescaler = 0;
    s_tim[id].Init.CounterMode = TIM_COUNTERMODE_UP;
    s_tim[id].Init.Period = 0xFFFF;
    s_tim[id].Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    s_tim[id].Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    enc.EncoderMode = TIM_ENCODERMODE_TI12;
    enc.IC1Polarity = TIM_ICPOLARITY_RISING;
    enc.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC1Prescaler = TIM_ICPSC_DIV1;
    enc.IC1Filter = 6;   /* 數位濾波，濾掉馬達的 PWM 雜訊 */
    enc.IC2Polarity = TIM_ICPOLARITY_RISING;
    enc.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    enc.IC2Prescaler = TIM_ICPSC_DIV1;
    enc.IC2Filter = 6;
    HAL_TIM_Encoder_Init(&s_tim[id], &enc);
    HAL_TIM_Encoder_Start(&s_tim[id], TIM_CHANNEL_ALL);
}

void encoder_init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_TIM5_CLK_ENABLE();
    for (int i = 0; i < ENC_COUNT; i++) {
        init_one((encoder_id_t)i);
    }
    encoder_reset();
}

void encoder_poll(int32_t counts[ENC_COUNT])
{
    for (int i = 0; i < ENC_COUNT; i++) {
        const uint16_t now = (uint16_t)k_hw[i].tim->CNT;
        s_total[i] += (int16_t)(uint16_t)(now - s_last[i]);
        s_last[i] = now;
    }
    if (counts != NULL) {
        memcpy(counts, s_total, sizeof s_total);
    }
}

void encoder_reset(void)
{
    for (int i = 0; i < ENC_COUNT; i++) {
        s_last[i] = (uint16_t)k_hw[i].tim->CNT;
        s_total[i] = 0;
    }
}

const char *encoder_name(encoder_id_t id)
{
    static const char *const names[ENC_COUNT] = {"A", "B", "C", "D"};
    return names[id];
}
