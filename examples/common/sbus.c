#include "sbus.h"

#include <string.h>
#include "board.h"

#define SBUS_BAUD            100000U
#define FRAME_LEN            25U
#define FRAME_GAP_US         2000U
#define HEADER               0x0FU
#define FLAG_FRAME_LOST      0x04U
#define FLAG_FAILSAFE        0x08U
#define STICK_MID            992U
#define STICK_DEADBAND       25U
#define STICK_RANGE          800.0f
#define SIGNAL_LOST_TIMEOUT  100U

static UART_HandleTypeDef s_uart;
static volatile uint8_t s_buf[FRAME_LEN];
static volatile uint32_t s_index;
static volatile uint32_t s_last_byte_us;
static volatile sbus_state_t s_state;

static void sbus_decode(const volatile uint8_t *b)
{
    uint32_t bits = 0, nbits = 0, byte = 1;

    for (unsigned c = 0; c < SBUS_CHANNELS; c++) {
        while (nbits < 11U) {
            bits |= (uint32_t)b[byte++] << nbits;
            nbits += 8U;
        }
        s_state.ch[c] = (uint16_t)(bits & 0x7FFU);
        bits >>= 11;
        nbits -= 11U;
    }
    const uint8_t flags = b[23];
    s_state.frame_lost = (flags & FLAG_FRAME_LOST) != 0U;
    s_state.failsafe = (flags & FLAG_FAILSAFE) != 0U;
    s_state.last_rx_ms = HAL_GetTick();
}

void UART5_IRQHandler(void)
{
    const uint32_t sr = SBUS_UART->SR;
    const uint8_t data = (uint8_t)SBUS_UART->DR;
    const uint32_t now = micros();

    if (sr & (USART_SR_PE | USART_SR_FE | USART_SR_NE | USART_SR_ORE)) {
        s_index = 0;
        return;
    }
    if (!(sr & USART_SR_RXNE)) {
        return;
    }
    if (now - s_last_byte_us > FRAME_GAP_US) {
        s_index = 0;
    }
    s_last_byte_us = now;

    if (s_index == 0 && data != HEADER) {
        return;
    }
    s_buf[s_index++] = data;
    if (s_index == FRAME_LEN) {
        s_index = 0;
        if (s_buf[24] == 0x00U || (s_buf[24] & 0x0FU) == 0x04U) {
            sbus_decode(s_buf);
        }
    }
}

void sbus_init(void)
{
    GPIO_InitTypeDef io = {0};

    __HAL_RCC_UART5_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    io.Pin = SBUS_RX_PIN;
    io.Mode = GPIO_MODE_AF_PP;
    io.Pull = GPIO_PULLUP;
    io.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    io.Alternate = SBUS_AF;
    HAL_GPIO_Init(SBUS_RX_PORT, &io);

    s_uart.Instance = SBUS_UART;
    s_uart.Init.BaudRate = SBUS_BAUD;
    s_uart.Init.WordLength = UART_WORDLENGTH_9B; /* 8 位元資料 + 偶同位 = 9 位元 */
    s_uart.Init.StopBits = UART_STOPBITS_2;
    s_uart.Init.Parity = UART_PARITY_EVEN;
    s_uart.Init.Mode = UART_MODE_RX;
    s_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    s_uart.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&s_uart);

    __HAL_UART_ENABLE_IT(&s_uart, UART_IT_RXNE);
    HAL_NVIC_SetPriority(UART5_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
}

bool sbus_read(sbus_state_t *out_state)
{
    const uint32_t now = HAL_GetTick();
    const bool alive = (now - s_state.last_rx_ms <= SIGNAL_LOST_TIMEOUT);

    if (out_state != NULL) {
        __disable_irq();
        *out_state = s_state;
        __enable_irq();
    }
    return alive && !s_state.failsafe && !s_state.frame_lost;
}

bool sbus_get_velocity(const sbus_state_t *state, float max_vx, float max_wz,
                       float *out_vx, float *out_wz, bool *out_active)
{
    /* 通道 1 (ch[0]) = 轉向，通道 2 (ch[1]) = 油門 */
    int32_t raw_turn = (int32_t)state->ch[0] - (int32_t)STICK_MID;
    int32_t raw_throttle = (int32_t)state->ch[1] - (int32_t)STICK_MID;

    bool active = false;

    if (raw_turn > (int32_t)STICK_DEADBAND) {
        raw_turn -= (int32_t)STICK_DEADBAND;
        active = true;
    } else if (raw_turn < -(int32_t)STICK_DEADBAND) {
        raw_turn += (int32_t)STICK_DEADBAND;
        active = true;
    } else {
        raw_turn = 0;
    }

    if (raw_throttle > (int32_t)STICK_DEADBAND) {
        raw_throttle -= (int32_t)STICK_DEADBAND;
        active = true;
    } else if (raw_throttle < -(int32_t)STICK_DEADBAND) {
        raw_throttle += (int32_t)STICK_DEADBAND;
        active = true;
    } else {
        raw_throttle = 0;
    }

    float norm_turn = -(float)raw_turn / STICK_RANGE;          /* 逆時針為正 */
    float norm_throttle = (float)raw_throttle / STICK_RANGE;   /* 前進為正 */

    if (norm_turn > 1.0f) norm_turn = 1.0f;
    if (norm_turn < -1.0f) norm_turn = -1.0f;
    if (norm_throttle > 1.0f) norm_throttle = 1.0f;
    if (norm_throttle < -1.0f) norm_throttle = -1.0f;

    if (out_vx != NULL) *out_vx = norm_throttle * max_vx;
    if (out_wz != NULL) *out_wz = norm_turn * max_wz;
    if (out_active != NULL) *out_active = active;

    return active;
}
