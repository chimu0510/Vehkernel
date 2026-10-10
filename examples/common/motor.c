#include "motor.h"

#include "board.h"

#define PWM_FREQ_HZ  20000U
#define PWM_PERIOD   (BOARD_SYSCLK_HZ / PWM_FREQ_HZ)   /* TIM8 在 APB2，計數時脈 168 MHz */

typedef struct {
    GPIO_TypeDef *pwm_port;
    uint16_t pwm_pin;
    uint32_t channel;
    GPIO_TypeDef *in1_port;
    uint16_t in1_pin;
    GPIO_TypeDef *in2_port;
    uint16_t in2_pin;
} motor_hw_t;

static const motor_hw_t k_hw[MOTOR_COUNT] = {
    {MOTOR_A_PWM_PORT, MOTOR_A_PWM_PIN, MOTOR_A_PWM_CH,
     MOTOR_A_IN1_PORT, MOTOR_A_IN1_PIN, MOTOR_A_IN2_PORT, MOTOR_A_IN2_PIN},
    {MOTOR_B_PWM_PORT, MOTOR_B_PWM_PIN, MOTOR_B_PWM_CH,
     MOTOR_B_IN1_PORT, MOTOR_B_IN1_PIN, MOTOR_B_IN2_PORT, MOTOR_B_IN2_PIN},
};

static TIM_HandleTypeDef s_tim;

void motor_init(void)
{
    GPIO_InitTypeDef io = {0};
    TIM_OC_InitTypeDef oc = {0};

    /* 先把 IN 腳設成低（脫機），再開 PWM。 */
    io.Mode = GPIO_MODE_OUTPUT_PP;
    io.Pull = GPIO_NOPULL;
    io.Speed = GPIO_SPEED_FREQ_LOW;
    for (int i = 0; i < MOTOR_COUNT; i++) {
        HAL_GPIO_WritePin(k_hw[i].in1_port, k_hw[i].in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(k_hw[i].in2_port, k_hw[i].in2_pin, GPIO_PIN_RESET);
        io.Pin = k_hw[i].in1_pin;
        HAL_GPIO_Init(k_hw[i].in1_port, &io);
        io.Pin = k_hw[i].in2_pin;
        HAL_GPIO_Init(k_hw[i].in2_port, &io);
    }

    __HAL_RCC_TIM8_CLK_ENABLE();
    s_tim.Instance = MOTOR_PWM_TIM;
    s_tim.Init.Prescaler = 0;
    s_tim.Init.CounterMode = TIM_COUNTERMODE_UP;
    s_tim.Init.Period = PWM_PERIOD - 1U;
    s_tim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    s_tim.Init.RepetitionCounter = 0;
    s_tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&s_tim);

    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    oc.OCIdleState = TIM_OCIDLESTATE_RESET;
    for (int i = 0; i < MOTOR_COUNT; i++) {
        HAL_TIM_PWM_ConfigChannel(&s_tim, &oc, k_hw[i].channel);
    }

    io.Mode = GPIO_MODE_AF_PP;
    io.Alternate = MOTOR_PWM_AF;
    for (int i = 0; i < MOTOR_COUNT; i++) {
        io.Pin = k_hw[i].pwm_pin;
        HAL_GPIO_Init(k_hw[i].pwm_port, &io);
        HAL_TIM_PWM_Start(&s_tim, k_hw[i].channel); /* 進階定時器，HAL 會一併打開 MOE */
    }
}

void motor_set(motor_id_t id, float duty)
{
    const motor_hw_t *hw = &k_hw[id];

    if (duty > 1.0f) {
        duty = 1.0f;
    } else if (duty < -1.0f) {
        duty = -1.0f;
    }
    const uint32_t pulse = (uint32_t)((duty < 0.0f ? -duty : duty) * (float)PWM_PERIOD);
    __HAL_TIM_SET_COMPARE(&s_tim, hw->channel, pulse);
    HAL_GPIO_WritePin(hw->in1_port, hw->in1_pin, duty > 0.0f ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(hw->in2_port, hw->in2_pin, duty < 0.0f ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void motor_stop_all(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++) {
        motor_set((motor_id_t)i, 0.0f);
    }
}
