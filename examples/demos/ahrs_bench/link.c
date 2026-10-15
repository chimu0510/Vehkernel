#include "link.h"

#include "board.h"
#include "console.h"
#include "frame.h"
#include "bench_proto.h"

#define TX_SIZE  2048U
#define RX_SIZE  512U
#define DMA_CH4  (4U << DMA_SxCR_CHSEL_Pos)

/* USART1：TX = DMA2 Stream7 Channel4，RX = DMA2 Stream2 Channel4。 */
#define TX_STREAM DMA2_Stream7
#define RX_STREAM DMA2_Stream2
#define TX_FLAGS  (DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CFEIF7)

static UART_HandleTypeDef s_uart;
static uint8_t s_tx[2][TX_SIZE];
static size_t s_tx_len[2];
static unsigned s_fill;               /* 正在累積的緩衝 */
static bool s_tx_busy;
static uint8_t s_rx[RX_SIZE];
static size_t s_rx_tail;
static uint8_t s_frame[FRAME_MAX_ENCODED(LINK_MAX_PAYLOAD)];
static size_t s_frame_len;
static bool s_frame_overflow;
static link_stats_t s_stats;

void link_init(void)
{
    /* GPIO 已由 console_init 設定為 USART1，這裡只改鮑率並接上 DMA。 */
    while (!(USART1->SR & USART_SR_TC)) {
    }
    uart_init_8n1(&s_uart, USART1, BENCH_BAUD);
    __HAL_RCC_DMA2_CLK_ENABLE();

    RX_STREAM->CR = 0;
    while (RX_STREAM->CR & DMA_SxCR_EN) {
    }
    DMA2->LIFCR = DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTEIF2 | DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CFEIF2;
    RX_STREAM->PAR = (uint32_t)&USART1->DR;
    RX_STREAM->M0AR = (uint32_t)s_rx;
    RX_STREAM->NDTR = RX_SIZE;
    RX_STREAM->CR = DMA_CH4 | DMA_SxCR_MINC | DMA_SxCR_CIRC;   /* 周邊 → 記憶體 */
    RX_STREAM->CR |= DMA_SxCR_EN;

    TX_STREAM->CR = 0;
    while (TX_STREAM->CR & DMA_SxCR_EN) {
    }
    TX_STREAM->PAR = (uint32_t)&USART1->DR;
    TX_STREAM->CR = DMA_CH4 | DMA_SxCR_MINC | DMA_SxCR_DIR_0;  /* 記憶體 → 周邊 */

    USART1->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT;
}

static void tx_pump(void)
{
    if (s_tx_busy) {
        if (TX_STREAM->CR & DMA_SxCR_EN) {
            return;
        }
        s_tx_busy = false;
    }
    if (s_tx_len[s_fill] == 0U) {
        return;
    }
    DMA2->HIFCR = TX_FLAGS;
    TX_STREAM->M0AR = (uint32_t)s_tx[s_fill];
    TX_STREAM->NDTR = (uint32_t)s_tx_len[s_fill];
    TX_STREAM->CR |= DMA_SxCR_EN;
    s_tx_busy = true;
    s_fill ^= 1U;
    s_tx_len[s_fill] = 0U;
}

bool link_send(const void *payload, size_t len)
{
    size_t *used = &s_tx_len[s_fill];

    if (len > LINK_MAX_PAYLOAD || *used + FRAME_MAX_ENCODED(len) > TX_SIZE) {
        s_stats.tx_full++;
        tx_pump();
        return false;
    }
    *used += frame_encode(payload, len, &s_tx[s_fill][*used]);
    s_stats.tx_frames++;
    tx_pump();
    return true;
}

size_t link_poll(uint8_t out[LINK_MAX_PAYLOAD])
{
    tx_pump();

    const size_t head = RX_SIZE - RX_STREAM->NDTR;
    while (s_rx_tail != head) {
        const uint8_t c = s_rx[s_rx_tail];
        s_rx_tail = (s_rx_tail + 1U) % RX_SIZE;
        if (c != 0U) {
            if (s_frame_len < sizeof s_frame) {
                s_frame[s_frame_len++] = c;
            } else {
                s_frame_overflow = true;
            }
            continue;
        }
        const size_t n = s_frame_len;
        const bool overflow = s_frame_overflow;
        s_frame_len = 0;
        s_frame_overflow = false;
        if (n == 0U) {
            continue;
        }
        const int len = overflow ? -1 : frame_decode(s_frame, n);
        if (len <= 0 || (size_t)len > LINK_MAX_PAYLOAD) {
            s_stats.rx_errors++;
            continue;
        }
        for (int i = 0; i < len; i++) {
            out[i] = s_frame[i];
        }
        s_stats.rx_frames++;
        return (size_t)len;
    }
    return 0;
}

link_stats_t link_stats(void)
{
    return s_stats;
}
