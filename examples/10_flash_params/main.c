/*
 * 10 Flash 參數儲存：驗證能在內部 Flash 保存設定並在斷電後保留，
 * 這是 ADR-0001「車頭方向存在 C50C 的 Flash」的前提。
 *
 * 使用最後一個 sector（sector 7，0x08060000，128 KB），範例程式只佔前兩個 sector，
 * 用串口或 SWD 燒錄範例時只會抹除用到的 sector，所以這裡的資料會保留下來；
 * 全片抹除（CubeProgrammer -e all）才會清掉。
 *
 *   1. 開機時讀取上一次寫入的紀錄；有效就代表資料跨越重設與斷電保留下來。
 *   2. 抹除 sector、寫入 boot_count + 1 的新紀錄、讀回並以 CRC 驗證。
 * 第一次執行沒有舊紀錄，「persisted」會是 SKIP；斷電（拔電池與 USB）後再跑一次應為 PASS。
 */
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "report.h"

#define PARAM_SECTOR       FLASH_SECTOR_7
#define PARAM_ADDR         0x08060000U
#define PARAM_SECTOR_SIZE  (128U * 1024U)
#define PARAM_MAGIC        0x43353043U   /* "C50C" */
#define PARAM_VERSION      1U

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t boot_count;
    uint32_t crc;          /* 前面欄位的 CRC-32 */
} param_record_t;

static uint32_t crc32(const void *data, size_t len)
{
    const uint8_t *p = data;
    uint32_t crc = 0xFFFFFFFFU;

    while (len--) {
        crc ^= *p++;
        for (int k = 0; k < 8; k++) {
            crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
        }
    }
    return ~crc;
}

static bool record_valid(const param_record_t *r)
{
    return r->magic == PARAM_MAGIC && r->version == PARAM_VERSION &&
           r->crc == crc32(r, offsetof(param_record_t, crc));
}

static bool erase_sector(uint32_t *elapsed_ms)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t bad_sector = 0;

    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = PARAM_SECTOR;
    erase.NbSectors = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;   /* VDD 2.7–3.6 V，可用 32 位元寫入 */

    const uint32_t t0 = HAL_GetTick();
    HAL_FLASH_Unlock();
    const HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&erase, &bad_sector);
    HAL_FLASH_Lock();
    *elapsed_ms = HAL_GetTick() - t0;
    return st == HAL_OK;
}

static bool sector_blank(void)
{
    const uint32_t *p = (const uint32_t *)PARAM_ADDR;
    for (uint32_t i = 0; i < PARAM_SECTOR_SIZE / 4U; i++) {
        if (p[i] != 0xFFFFFFFFU) {
            return false;
        }
    }
    return true;
}

static bool write_record(const param_record_t *r)
{
    const uint32_t *words = (const uint32_t *)r;
    bool ok = true;

    HAL_FLASH_Unlock();
    for (uint32_t i = 0; i < sizeof *r / 4U && ok; i++) {
        ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PARAM_ADDR + 4U * i, words[i]) == HAL_OK;
    }
    HAL_FLASH_Lock();
    return ok;
}

int main(void)
{
    board_init();
    report_begin("10_flash_params");

    const param_record_t *stored = (const param_record_t *)PARAM_ADDR;
    uint32_t boot_count = 0;
    if (record_valid(stored)) {
        boot_count = stored->boot_count;
        report_check(true, "persisted", "record from the previous run found, boot_count = %lu",
                     (unsigned long)boot_count);
    } else {
        report_skip("persisted", "no valid record yet (first run, or the chip was mass-erased)");
    }

    uint32_t erase_ms = 0;
    if (!report_check(erase_sector(&erase_ms), "erase", "sector 7 (128 KB) in %lu ms",
                      (unsigned long)erase_ms) ||
        !report_check(sector_blank(), "blank check", "all 128 KB read back as 0xFF")) {
        report_end();
        report_halt();
    }

    param_record_t rec = {PARAM_MAGIC, PARAM_VERSION, boot_count + 1U, 0};
    rec.crc = crc32(&rec, offsetof(param_record_t, crc));
    const uint32_t t0 = micros();
    const bool written = write_record(&rec);
    const uint32_t write_us = micros() - t0;
    report_check(written, "program", "%u bytes in %lu us", (unsigned)sizeof rec, (unsigned long)write_us);
    report_check(memcmp(stored, &rec, sizeof rec) == 0 && record_valid(stored), "read back",
                 "boot_count = %lu, crc 0x%08lX", (unsigned long)stored->boot_count,
                 (unsigned long)stored->crc);

    printf("  now power-cycle the board (unplug battery and USB), then run: tools/c50c.py monitor --reset\n"
           "  'persisted' should PASS with boot_count = %lu\n", (unsigned long)rec.boot_count);
    report_end();
    report_halt();
}
