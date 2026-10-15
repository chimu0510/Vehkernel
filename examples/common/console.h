/*
 * USART1 主控台（Type-C 串口 1，115200 8N1）。printf 經由 _write 輸出到這裡。
 */
#ifndef CONSOLE_H
#define CONSOLE_H

#include <stddef.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

typedef enum {
    ANSWER_NO = 0,
    ANSWER_YES,
    ANSWER_TIMEOUT,
} answer_t;

void console_init(void);
void console_write(const uint8_t *data, size_t len);

/* 讀一個字元；逾時回傳 -1。 */
int console_getc(uint32_t timeout_ms);

/* 丟棄已收到但尚未讀取的字元。 */
void console_flush_input(void);

/* 讀一行（Enter 結束、回顯輸入）；回傳長度，逾時回傳 -1。 */
int console_readline(char *buf, size_t size, uint32_t timeout_ms);

/* 印出問題並等待 y/n；逾時回傳 ANSWER_TIMEOUT。 */
answer_t console_ask(const char *question, uint32_t timeout_ms);

/* 供其他範例共用的 UART 初始化（8N1、無流量控制）。 */
void uart_init_8n1(UART_HandleTypeDef *huart, USART_TypeDef *instance, uint32_t baud);

#endif /* CONSOLE_H */
