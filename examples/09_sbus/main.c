/*
 * 09 SBUS 接收（CN7：1 = 訊號、2 = 5V、3 = GND；PD2 = UART5_RX，100000 baud 8E2）。
 * 板上已用 NPN 把 SBUS 的反相訊號轉回來，UART 以一般極性接收。
 *   1. 收到完整訊框、訊框率、錯誤比例、failsafe 旗標。
 *   2. 搖桿：15 秒內把兩支搖桿推到四個角落，通道 1–4 都要到達兩端。
 *   3. 失效保護：關掉遙控器，確認接收機的行為（送出 failsafe 旗標或停止輸出），
 *      再打開遙控器確認恢復。這決定韌體要怎麼偵測遙控器失聯。
 *   4. 之後以 5 Hz 持續輸出通道 1–8 與旗標。
 * 執行前先把接收機與遙控器對頻好，並打開遙控器。
 */
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "report.h"

#define SBUS_BAUD          100000U
#define FRAME_LEN          25U
#define FRAME_GAP_US       2000U    /* 訊框之間至少 3 ms 空檔，以 2 ms 判斷新訊框開始 */
#define HEADER             0x0FU
#define FLAG_FRAME_LOST    0x04U
#define FLAG_FAILSAFE      0x08U
#define CHANNELS           16U
#define STICK_LOW          500U     /* 11 位元原始值；常見範圍約 172..1811 */
#define STICK_HIGH         1500U
#define STICK_TIMEOUT_MS   15000U
#define FAILSAFE_TIMEOUT_MS 20000U
#define SIGNAL_LOST_MS     100U

typedef struct {
    uint16_t ch[CHANNELS];
    uint8_t flags;
} sbus_frame_t;

static UART_HandleTypeDef s_uart;

/* 由中斷維護的狀態 */
static volatile uint8_t s_buf[FRAME_LEN];
static volatile uint32_t s_index;
static volatile uint32_t s_last_byte_us;
static volatile sbus_frame_t s_frame;
static volatile uint32_t s_frame_count;
static volatile uint32_t s_bad_frames;
static volatile uint32_t s_uart_errors;
static volatile uint32_t s_last_frame_ms;

static void decode(const volatile uint8_t *b)
{
    uint32_t bits = 0, nbits = 0, byte = 1;

    for (unsigned c = 0; c < CHANNELS; c++) {
        while (nbits < 11U) {
            bits |= (uint32_t)b[byte++] << nbits;
            nbits += 8U;
        }
        s_frame.ch[c] = (uint16_t)(bits & 0x7FFU);
        bits >>= 11;
        nbits -= 11U;
    }
    s_frame.flags = b[23];
}

void UART5_IRQHandler(void)
{
    const uint32_t sr = SBUS_UART->SR;
    const uint8_t data = (uint8_t)SBUS_UART->DR;  /* 讀 SR 再讀 DR 會清除錯誤旗標 */
    const uint32_t now = micros();

    if (sr & (USART_SR_PE | USART_SR_FE | USART_SR_NE | USART_SR_ORE)) {
        s_uart_errors++;
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
        /* 結尾為 0x00；SBUS2 接收機會用 0x04/0x14/0x24/0x34 輪替。 */
        if (s_buf[24] == 0x00U || (s_buf[24] & 0x0FU) == 0x04U) {
            decode(s_buf);
            s_frame_count++;
            s_last_frame_ms = HAL_GetTick();
        } else {
            s_bad_frames++;
        }
    }
}

static void sbus_init(void)
{
    GPIO_InitTypeDef io = {0};

    __HAL_RCC_UART5_CLK_ENABLE();
    io.Mode = GPIO_MODE_AF_PP;
    io.Pull = GPIO_PULLUP;
    io.Speed = GPIO_SPEED_FREQ_HIGH;
    io.Alternate = SBUS_AF;
    io.Pin = SBUS_RX_PIN;
    HAL_GPIO_Init(SBUS_RX_PORT, &io);

    s_uart.Instance = SBUS_UART;
    s_uart.Init.BaudRate = SBUS_BAUD;
    s_uart.Init.WordLength = UART_WORDLENGTH_9B;   /* 8 資料位元 + 同位位元 */
    s_uart.Init.StopBits = UART_STOPBITS_2;
    s_uart.Init.Parity = UART_PARITY_EVEN;
    s_uart.Init.Mode = UART_MODE_RX;
    s_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    s_uart.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&s_uart);

    SBUS_UART->CR1 |= USART_CR1_RXNEIE;
    HAL_NVIC_SetPriority(UART5_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
}

static void snapshot(sbus_frame_t *f)
{
    __disable_irq();
    memcpy(f, (const void *)&s_frame, sizeof *f);
    __enable_irq();
}

static bool signal_present(void)
{
    return s_frame_count > 0 && HAL_GetTick() - s_last_frame_ms < SIGNAL_LOST_MS;
}

static bool test_link(void)
{
    HAL_Delay(3000);
    if (!report_check(s_frame_count > 0, "frames", "%lu frames in 3 s",
                      (unsigned long)s_frame_count)) {
        printf("  check: receiver bound and powered (CN7 pin 2 = 5 V), signal on CN7 pin 1,"
               " transmitter ON, %lu UART errors\n", (unsigned long)s_uart_errors);
        return false;
    }

    const uint32_t f0 = s_frame_count, b0 = s_bad_frames, e0 = s_uart_errors;
    HAL_Delay(2000);
    const uint32_t frames = s_frame_count - f0;
    const uint32_t bad = s_bad_frames - b0 + (s_uart_errors - e0);
    report_check(frames >= 80U, "frame rate", "%lu frames/s (~%.1f ms period)",
                 (unsigned long)(frames / 2U), frames ? (double)(2000.0f / (float)frames) : 0.0);
    report_check(bad * 20U < frames + bad, "frame errors", "%lu bad of %lu (expect < 5%%)",
                 (unsigned long)bad, (unsigned long)(frames + bad));

    sbus_frame_t f;
    snapshot(&f);
    report_check(!(f.flags & FLAG_FAILSAFE), "failsafe flag", "clear while the transmitter is on (flags 0x%02X)",
                 f.flags);
    return true;
}

static void test_sticks(void)
{
    uint16_t lo[4] = {2047, 2047, 2047, 2047}, hi[4] = {0, 0, 0, 0};
    const uint32_t start = HAL_GetTick();

    printf("  move BOTH sticks to all four corners within %lus\n",
           (unsigned long)(STICK_TIMEOUT_MS / 1000U));
    bool done = false;
    while (!done && HAL_GetTick() - start < STICK_TIMEOUT_MS) {
        sbus_frame_t f;
        snapshot(&f);
        done = true;
        for (int c = 0; c < 4; c++) {
            lo[c] = f.ch[c] < lo[c] ? f.ch[c] : lo[c];
            hi[c] = f.ch[c] > hi[c] ? f.ch[c] : hi[c];
            done &= lo[c] < STICK_LOW && hi[c] > STICK_HIGH;
        }
        HAL_Delay(10);
    }
    for (int c = 0; c < 4; c++) {
        char name[8];
        snprintf(name, sizeof name, "ch%d", c + 1);
        report_check(lo[c] < STICK_LOW && hi[c] > STICK_HIGH, name, "range %u..%u (expect < %u and > %u)",
                     lo[c], hi[c], STICK_LOW, STICK_HIGH);
    }
}

static void test_failsafe(void)
{
    uint32_t start = HAL_GetTick();
    bool flagged = false, silent = false;

    printf("  turn the transmitter OFF within %lus\n", (unsigned long)(FAILSAFE_TIMEOUT_MS / 1000U));
    while (!flagged && !silent && HAL_GetTick() - start < FAILSAFE_TIMEOUT_MS) {
        sbus_frame_t f;
        snapshot(&f);
        flagged = signal_present() && (f.flags & FLAG_FAILSAFE);
        silent = !signal_present();
        HAL_Delay(5);
    }
    if (!report_check(flagged || silent, "transmitter off", "%s",
                      flagged ? "receiver keeps sending frames with the FAILSAFE flag set"
                      : silent ? "receiver stops sending frames"
                               : "no failsafe flag and frames continue")) {
        return;
    }

    printf("  turn the transmitter back ON within %lus\n", (unsigned long)(FAILSAFE_TIMEOUT_MS / 1000U));
    start = HAL_GetTick();
    bool recovered = false;
    while (!recovered && HAL_GetTick() - start < FAILSAFE_TIMEOUT_MS) {
        sbus_frame_t f;
        snapshot(&f);
        recovered = signal_present() && !(f.flags & FLAG_FAILSAFE);
        HAL_Delay(5);
    }
    report_check(recovered, "transmitter on", "frames without failsafe flag again");
}

static void stream(void)
{
    printf("  streaming at 5 Hz: ch1..ch8, flags (0x04 frame lost, 0x08 failsafe)\n");
    for (;;) {
        sbus_frame_t f;
        snapshot(&f);
        printf("  %4u %4u %4u %4u %4u %4u %4u %4u  flags 0x%02X%s\n",
               f.ch[0], f.ch[1], f.ch[2], f.ch[3], f.ch[4], f.ch[5], f.ch[6], f.ch[7], f.flags,
               signal_present() ? "" : "  NO SIGNAL");
        HAL_Delay(200);
    }
}

int main(void)
{
    board_init();
    report_begin("09_sbus");
    sbus_init();

    if (test_link()) {
        test_sticks();
        test_failsafe();
    }
    report_end();
    stream();
}
