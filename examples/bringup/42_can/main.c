/*
 * 42 CAN1（PD0 = RX、PD1 = TX，板載 VP230 收發器），500 kbit/s。
 *   1. 靜默迴環（silent loopback）：只測 CAN 控制器本身，不碰匯流排。
 *   2. 迴環（loopback）：TX 實際送到收發器，RX 在內部接回；確認腳位設定沒有讓控制器出錯。
 *   3. 外部節點（可選）：接一個 500 kbit/s 的 USB-CAN 轉接器，送出任一標準訊框，
 *      例如 Linux 上的 `cansend can0 123#DEADBEEF`；板子會以 ID + 1 回送相同資料。
 *      只有這一步會驗證到 VP230 與 CANH/CANL 接線。匯流排兩端需要 120 Ω 終端電阻。
 */
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "report.h"

#define TEST_ID            0x123U
#define EXTERNAL_TIMEOUT_MS 30000U

static CAN_HandleTypeDef s_can;

/* APB1 42 MHz / (6 × (1 + 11 + 2) tq) = 500 kbit/s，取樣點 85.7%。 */
static bool can_start(uint32_t mode)
{
    CAN_FilterTypeDef filter = {0};

    if (s_can.State != HAL_CAN_STATE_RESET) {
        HAL_CAN_DeInit(&s_can);
    }
    s_can.Instance = CAN1;
    s_can.Init.Prescaler = 6;
    s_can.Init.Mode = mode;
    s_can.Init.SyncJumpWidth = CAN_SJW_1TQ;
    s_can.Init.TimeSeg1 = CAN_BS1_11TQ;
    s_can.Init.TimeSeg2 = CAN_BS2_2TQ;
    s_can.Init.TimeTriggeredMode = DISABLE;
    s_can.Init.AutoBusOff = ENABLE;
    s_can.Init.AutoWakeUp = DISABLE;
    s_can.Init.AutoRetransmission = DISABLE;   /* 沒有其他節點 ACK 時不要無限重送 */
    s_can.Init.ReceiveFifoLocked = DISABLE;
    s_can.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&s_can) != HAL_OK) {
        return false;
    }

    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0;
    filter.FilterIdLow = 0;
    filter.FilterMaskIdHigh = 0;            /* 全部接收 */
    filter.FilterMaskIdLow = 0;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;
    return HAL_CAN_ConfigFilter(&s_can, &filter) == HAL_OK && HAL_CAN_Start(&s_can) == HAL_OK;
}

static bool send(uint32_t id, const uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef tx = {0};
    uint32_t mailbox;

    tx.StdId = id;
    tx.IDE = CAN_ID_STD;
    tx.RTR = CAN_RTR_DATA;
    tx.DLC = len;
    return HAL_CAN_AddTxMessage(&s_can, &tx, (uint8_t *)data, &mailbox) == HAL_OK;
}

static bool receive(CAN_RxHeaderTypeDef *rx, uint8_t data[8], uint32_t timeout_ms)
{
    const uint32_t start = HAL_GetTick();
    while (HAL_CAN_GetRxFifoFillLevel(&s_can, CAN_RX_FIFO0) == 0U) {
        if (HAL_GetTick() - start >= timeout_ms) {
            return false;
        }
    }
    return HAL_CAN_GetRxMessage(&s_can, CAN_RX_FIFO0, rx, data) == HAL_OK;
}

static void test_loopback(uint32_t mode, const char *name)
{
    if (!report_check(can_start(mode), name, "init and start")) {
        return;
    }
    unsigned ok = 0;
    for (uint8_t i = 0; i < 3; i++) {
        const uint8_t tx[8] = {0xC5, 0x0C, i, 0x55, 0xAA, 0x00, 0xFF, (uint8_t)(0x10 + i)};
        CAN_RxHeaderTypeDef rx;
        uint8_t data[8];
        if (send(TEST_ID + i, tx, sizeof tx) && receive(&rx, data, 50) &&
            rx.StdId == TEST_ID + i && rx.DLC == 8U && memcmp(tx, data, 8) == 0) {
            ok++;
        }
    }
    report_check(ok == 3U, name, "%u/3 frames echoed intact (error code 0x%08lX)",
                 ok, (unsigned long)HAL_CAN_GetError(&s_can));
}

static void test_external(void)
{
    if (!report_check(can_start(CAN_MODE_NORMAL), "normal mode", "init and start")) {
        return;
    }
    printf("  optional: send a standard frame from a 500 kbit/s CAN adapter within %lus,\n"
           "  e.g. cansend can0 123#DEADBEEF; the board answers with ID + 1\n",
           (unsigned long)(EXTERNAL_TIMEOUT_MS / 1000U));

    CAN_RxHeaderTypeDef rx;
    uint8_t data[8];
    if (!receive(&rx, data, EXTERNAL_TIMEOUT_MS)) {
        report_skip("external node", "no frame received (no adapter connected?)");
        return;
    }
    const bool sent = rx.IDE == CAN_ID_STD && send((rx.StdId + 1U) & 0x7FFU, data, (uint8_t)rx.DLC);
    HAL_Delay(10);
    const uint32_t esr = CAN1->ESR;
    report_check(sent && ((esr >> 16) & 0xFFU) == 0U, "external node",
                 "received ID 0x%03lX DLC %lu, echoed as 0x%03lX (tx errors %lu)",
                 (unsigned long)rx.StdId, (unsigned long)rx.DLC,
                 (unsigned long)((rx.StdId + 1U) & 0x7FFU), (unsigned long)((esr >> 16) & 0xFFU));
}

int main(void)
{
    GPIO_InitTypeDef io = {0};

    board_init();
    report_begin("42_can");

    __HAL_RCC_CAN1_CLK_ENABLE();
    io.Mode = GPIO_MODE_AF_PP;
    io.Pull = GPIO_PULLUP;
    io.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    io.Alternate = CAN_AF;
    io.Pin = CAN_RX_PIN;
    HAL_GPIO_Init(CAN_RX_PORT, &io);
    io.Pin = CAN_TX_PIN;
    HAL_GPIO_Init(CAN_TX_PORT, &io);

    test_loopback(CAN_MODE_SILENT_LOOPBACK, "silent loopback");
    test_loopback(CAN_MODE_LOOPBACK, "loopback");
    test_external();

    report_end();
    report_halt();
}
