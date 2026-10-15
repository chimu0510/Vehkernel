#include "console.h"

#include <stdio.h>

#include "board.h"

static UART_HandleTypeDef s_console;

void uart_init_8n1(UART_HandleTypeDef *huart, USART_TypeDef *instance, uint32_t baud)
{
    huart->Instance = instance;
    huart->Init.BaudRate = baud;
    huart->Init.WordLength = UART_WORDLENGTH_8B;
    huart->Init.StopBits = UART_STOPBITS_1;
    huart->Init.Parity = UART_PARITY_NONE;
    huart->Init.Mode = UART_MODE_TX_RX;
    huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart->Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(huart);
}

void console_init(void)
{
    GPIO_InitTypeDef io = {0};

    __HAL_RCC_USART1_CLK_ENABLE();
    io.Mode = GPIO_MODE_AF_PP;
    io.Pull = GPIO_PULLUP;
    io.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    io.Alternate = CONSOLE_AF;
    io.Pin = CONSOLE_TX_PIN;
    HAL_GPIO_Init(CONSOLE_TX_PORT, &io);
    io.Pin = CONSOLE_RX_PIN;
    HAL_GPIO_Init(CONSOLE_RX_PORT, &io);

    uart_init_8n1(&s_console, CONSOLE_UART, CONSOLE_BAUD);

    /* 讓 printf 不經緩衝直接輸出，避免當機前最後幾行遺失。 */
    setvbuf(stdout, NULL, _IONBF, 0);
}

void console_write(const uint8_t *data, size_t len)
{
    HAL_UART_Transmit(&s_console, (uint8_t *)data, (uint16_t)len, HAL_MAX_DELAY);
}

int console_getc(uint32_t timeout_ms)
{
    uint8_t c;

    /* 先清掉溢位旗標，否則之前沒讀走的字元會讓接收卡住。 */
    __HAL_UART_CLEAR_OREFLAG(&s_console);
    if (HAL_UART_Receive(&s_console, &c, 1, timeout_ms) != HAL_OK) {
        return -1;
    }
    return c;
}

void console_flush_input(void)
{
    while (console_getc(0) >= 0) {
    }
}

int console_readline(char *buf, size_t size, uint32_t timeout_ms)
{
    const uint32_t start = HAL_GetTick();
    size_t len = 0;

    while (HAL_GetTick() - start < timeout_ms) {
        const int c = console_getc(10);
        if (c < 0) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            if (len == 0) {
                continue; /* 忽略前一行留下的 CR/LF */
            }
            printf("\r\n");
            buf[len] = '\0';
            return (int)len;
        }
        if ((c == '\b' || c == 0x7F) && len > 0) {
            len--;
            printf("\b \b");
            continue;
        }
        if (len + 1 < size && c >= 0x20 && c < 0x7F) {
            buf[len++] = (char)c;
            putchar(c);
        }
    }
    buf[len] = '\0';
    return -1;
}

answer_t console_ask(const char *question, uint32_t timeout_ms)
{
    const uint32_t start = HAL_GetTick();

    console_flush_input();
    printf("  ? %s [y/n] (%lus) ", question, (unsigned long)(timeout_ms / 1000U));
    while (HAL_GetTick() - start < timeout_ms) {
        const int c = console_getc(10);
        if (c == 'y' || c == 'Y') {
            printf("y\r\n");
            return ANSWER_YES;
        }
        if (c == 'n' || c == 'N') {
            printf("n\r\n");
            return ANSWER_NO;
        }
    }
    printf("(timeout)\r\n");
    return ANSWER_TIMEOUT;
}

/* newlib 的 printf 最終呼叫這裡；把 \n 轉成 \r\n 方便一般序列終端機顯示。 */
int _write(int fd, const char *ptr, int len)
{
    (void)fd;
    for (int i = 0; i < len; i++) {
        if (ptr[i] == '\n' && (i == 0 || ptr[i - 1] != '\r')) {
            console_write((const uint8_t *)"\r", 1);
        }
        console_write((const uint8_t *)&ptr[i], 1);
    }
    return len;
}
