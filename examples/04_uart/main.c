/*
 * 04 串口：測試兩個 Type-C 串口的收發。
 *   - USART1（PA9/PA10，主控台）：輸入 ping 後按 Enter。
 *   - USART3（PB10/PB11）：接上第二個 Type-C 口，用 115200 開終端機，輸入 ping 後按 Enter。
 * 兩者都會把收到的內容回顯，並在主控台回報結果。
 */
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "console.h"
#include "report.h"

#define LINE_TIMEOUT_MS 30000U
#define BANNER_PERIOD_MS 2000U

static UART_HandleTypeDef s_aux;

static void aux_init(void)
{
    GPIO_InitTypeDef io = {0};

    __HAL_RCC_USART3_CLK_ENABLE();
    io.Mode = GPIO_MODE_AF_PP;
    io.Pull = GPIO_PULLUP;
    io.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    io.Alternate = AUX_AF;
    io.Pin = AUX_TX_PIN;
    HAL_GPIO_Init(AUX_TX_PORT, &io);
    io.Pin = AUX_RX_PIN;
    HAL_GPIO_Init(AUX_RX_PORT, &io);
    uart_init_8n1(&s_aux, AUX_UART, CONSOLE_BAUD);
}

static void aux_puts(const char *s)
{
    HAL_UART_Transmit(&s_aux, (uint8_t *)s, (uint16_t)strlen(s), HAL_MAX_DELAY);
}

/* 在 USART3 上讀一行並回顯，期間定時送出提示；逾時回傳 -1。 */
static int aux_readline(char *buf, size_t size, uint32_t timeout_ms)
{
    const uint32_t start = HAL_GetTick();
    uint32_t last_banner = start - BANNER_PERIOD_MS;
    size_t len = 0;

    while (HAL_GetTick() - start < timeout_ms) {
        if (len == 0 && HAL_GetTick() - last_banner >= BANNER_PERIOD_MS) {
            aux_puts("\r\nC50C USART3: type ping and press Enter\r\n> ");
            last_banner = HAL_GetTick();
        }
        uint8_t c;
        __HAL_UART_CLEAR_OREFLAG(&s_aux);
        if (HAL_UART_Receive(&s_aux, &c, 1, 10) != HAL_OK) {
            continue;
        }
        if (c == '\r' || c == '\n') {
            if (len == 0) {
                continue;
            }
            buf[len] = '\0';
            return (int)len;
        }
        if (len + 1 < size && c >= 0x20 && c < 0x7F) {
            buf[len++] = (char)c;
            HAL_UART_Transmit(&s_aux, &c, 1, HAL_MAX_DELAY);
        }
    }
    buf[len] = '\0';
    return -1;
}

static void test_console(void)
{
    char line[32];

    printf("  USART1: type ping and press Enter within %lus\n> ",
           (unsigned long)(LINE_TIMEOUT_MS / 1000U));
    console_flush_input();
    const int n = console_readline(line, sizeof line, LINE_TIMEOUT_MS);
    if (n < 0) {
        printf("\n");
        report_check(false, "usart1 rx", "no line received");
        return;
    }
    report_check(strcmp(line, "ping") == 0, "usart1 rx", "received \"%s\" (%d bytes)", line, n);
}

static void test_aux(void)
{
    char line[32];

    printf("  USART3: open the second Type-C port at 115200, type ping and press Enter within %lus\n",
           (unsigned long)(LINE_TIMEOUT_MS / 1000U));
    aux_init();
    const int n = aux_readline(line, sizeof line, LINE_TIMEOUT_MS);
    if (n < 0) {
        report_check(false, "usart3 rx", "no line received");
        return;
    }
    aux_puts("\r\npong\r\n");
    report_check(strcmp(line, "ping") == 0, "usart3 rx", "received \"%s\" (%d bytes)", line, n);
    printf("  check the USART3 terminal shows \"pong\"\n");
}

int main(void)
{
    board_init();
    report_begin("04_uart");

    report_check(true, "usart1 tx", "you are reading this at %lu baud", (unsigned long)CONSOLE_BAUD);
    test_console();
    test_aux();

    report_end();
    report_halt();
}
