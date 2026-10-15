#include "ugv_link.h"

#include <string.h>
#include "board.h"
#include "console.h"

#define TX_SIZE   2048U
#define RX_SIZE   512U
#define DMA_CH4   (4U << DMA_SxCR_CHSEL_Pos)

#define TX_STREAM DMA2_Stream7
#define RX_STREAM DMA2_Stream2
#define TX_FLAGS  (DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CFEIF7)

static UART_HandleTypeDef s_uart;
static uint8_t s_tx[2][TX_SIZE];
static size_t s_tx_len[2];
static unsigned s_fill;
static bool s_tx_busy;
static uint8_t s_rx[RX_SIZE];
static size_t s_rx_tail;
static uint8_t s_frame[UGV_MAX_FRAME_SIZE];
static size_t s_frame_len;
static bool s_frame_overflow;
static ugv_link_stats_t s_stats;

void ugv_link_init(uint32_t baud)
{
    while (!(USART1->SR & USART_SR_TC)) {
    }
    uart_init_8n1(&s_uart, USART1, baud);
    __HAL_RCC_DMA2_CLK_ENABLE();

    RX_STREAM->CR = 0;
    while (RX_STREAM->CR & DMA_SxCR_EN) {
    }
    DMA2->LIFCR = DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTEIF2 | DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CFEIF2;
    RX_STREAM->PAR = (uint32_t)&USART1->DR;
    RX_STREAM->M0AR = (uint32_t)s_rx;
    RX_STREAM->NDTR = RX_SIZE;
    RX_STREAM->CR = DMA_CH4 | DMA_SxCR_MINC | DMA_SxCR_CIRC;   /* 周邊 -> 記憶體循環 */
    RX_STREAM->CR |= DMA_SxCR_EN;

    TX_STREAM->CR = 0;
    while (TX_STREAM->CR & DMA_SxCR_EN) {
    }
    TX_STREAM->PAR = (uint32_t)&USART1->DR;
    TX_STREAM->CR = DMA_CH4 | DMA_SxCR_MINC | DMA_SxCR_DIR_0;  /* 記憶體 -> 周邊 */

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

bool ugv_link_send(const void *payload, size_t len)
{
    size_t *used = &s_tx_len[s_fill];

    if (len > UGV_MAX_PAYLOAD_SIZE || *used + FRAME_MAX_ENCODED(len) > TX_SIZE) {
        s_stats.tx_full++;
        tx_pump();
        return false;
    }
    *used += ugv_encode_frame(payload, len, &s_tx[s_fill][*used]);
    s_stats.tx_frames++;
    tx_pump();
    return true;
}

size_t ugv_link_poll(ugv_msg_payload_t *out_msg)
{
    tx_pump();

    const size_t head = RX_SIZE - RX_STREAM->NDTR;
    while (s_rx_tail != head) {
        const uint8_t c = s_rx[s_rx_tail];
        s_rx_tail = (s_rx_tail + 1U) % RX_SIZE;
        if (c != 0U) {
            if (s_frame_len < sizeof(s_frame)) {
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
        if (overflow) {
            s_stats.rx_errors++;
            continue;
        }
        const int len = ugv_decode_frame(s_frame, n, out_msg);
        if (len <= 0) {
            s_stats.rx_errors++;
            continue;
        }
        s_stats.rx_frames++;
        return (size_t)len;
    }
    return 0;
}

ugv_link_stats_t ugv_link_get_stats(void)
{
    return s_stats;
}
