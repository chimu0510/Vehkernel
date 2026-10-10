/*
 * 01 板子資訊：確認 MCU、時脈、外部晶振、硬體版本與 IMU 型號。
 * 之後的範例都假設這個範例已經通過。
 */
#include <stdio.h>

#include "board.h"
#include "board_id.h"
#include "report.h"

#define STM32F40X_DEV_ID   0x413U
#define FLASH_SIZE_KB      (*(const uint16_t *)FLASHSIZE_BASE)

static const char *reset_cause(void)
{
    const uint32_t csr = RCC->CSR;
    __HAL_RCC_CLEAR_RESET_FLAGS();
    if (csr & RCC_CSR_LPWRRSTF) return "low-power";
    if (csr & RCC_CSR_WWDGRSTF) return "window watchdog";
    if (csr & RCC_CSR_IWDGRSTF) return "independent watchdog";
    if (csr & RCC_CSR_SFTRSTF)  return "software";
    if (csr & RCC_CSR_PORRSTF)  return "power-on";
    if (csr & RCC_CSR_BORRSTF)  return "brown-out";
    if (csr & RCC_CSR_PINRSTF)  return "reset pin";
    return "unknown";
}

int main(void)
{
    board_init();
    report_begin("01_board_info");

    const uint32_t dev_id = HAL_GetDEVID();
    const uint32_t uid0 = HAL_GetUIDw0();
    const uint32_t uid1 = HAL_GetUIDw1();
    const uint32_t uid2 = HAL_GetUIDw2();
    printf("  UID %08lX-%08lX-%08lX, rev 0x%04lX, flash %u KB, reset by %s\n",
           (unsigned long)uid2, (unsigned long)uid1, (unsigned long)uid0,
           (unsigned long)HAL_GetREVID(), FLASH_SIZE_KB, reset_cause());

    report_check(dev_id == STM32F40X_DEV_ID, "mcu",
                 "DEV_ID 0x%03lX (expect 0x413 STM32F405/407)", (unsigned long)dev_id);
    report_check(FLASH_SIZE_KB == 512U, "flash size", "%u KB (expect 512 KB, STM32F407VET6)",
                 FLASH_SIZE_KB);
    report_check(board_hse_ok(), "hse 8 MHz",
                 board_hse_ok() ? "crystal started" : "crystal failed, running on HSI");
    report_check(SystemCoreClock == BOARD_SYSCLK_HZ, "sysclk", "%lu Hz (expect %lu)",
                 (unsigned long)SystemCoreClock, (unsigned long)BOARD_SYSCLK_HZ);

    /* 用 SysTick（HAL tick）與 DWT 週期計數器互相比對，確認時脈設定一致。
     * 先對齊到 tick 邊緣再量剛好 100 個 tick；HAL_Delay(100) 會多等 1 tick。 */
    const uint32_t t_sync = HAL_GetTick();
    while (HAL_GetTick() == t_sync) {
    }
    const uint32_t t0 = HAL_GetTick();
    const uint32_t c0 = DWT->CYCCNT;
    while (HAL_GetTick() - t0 < 100U) {
    }
    const uint32_t cycles = DWT->CYCCNT - c0;
    const uint32_t mhz = (cycles + 50000U) / 100000U;
    report_check(mhz >= 166U && mhz <= 170U, "tick vs cpu clock",
                 "100 ms tick = %lu cycles (~%lu MHz)", (unsigned long)cycles, (unsigned long)mhz);

    const board_id_t id = board_id_detect();
    report_check(id.imu != IMU_NONE, "imu", "%s (WHO_AM_I 0x%02X)",
                 board_imu_name(id.imu), id.imu_who_am_i);
    report_check(id.version != HW_UNKNOWN, "hardware version",
                 "%s (vendor naming), version pins PC13/PE3/PE2 = %u%u%u",
                 board_version_name(id.version),
                 (id.version_bits >> 2) & 1U, (id.version_bits >> 1) & 1U, id.version_bits & 1U);
    if (id.version == HW_V1_2) {
        printf("  note: V1.2 has RS485 on PD5/PD6 (D50A); OLED uses PD7/PD4/PE1/PE0\n");
    }

    report_end();
    report_halt();
}
