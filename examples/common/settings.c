#include "settings.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "board.h"

#define SETTINGS_SECTOR    FLASH_SECTOR_7
#define SETTINGS_ADDR      0x08060000U
#define SETTINGS_MAGIC     0x53303543U   /* "C50S"；13_flash_params 的紀錄是 "C50C" */
#define SETTINGS_VERSION   1U

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t heading;
    uint32_t crc;          /* 前面欄位的 CRC-32 */
} record_t;

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

static bool record_valid(const record_t *r)
{
    return r->magic == SETTINGS_MAGIC && r->version == SETTINGS_VERSION &&
           r->heading <= HEADING_OMNI_END_FRONT &&
           r->crc == crc32(r, offsetof(record_t, crc));
}

bool settings_load(settings_t *out)
{
    const record_t *r = (const record_t *)SETTINGS_ADDR;

    out->heading = HEADING_DRIVE_FRONT;
    if (!record_valid(r)) {
        return false;
    }
    out->heading = (heading_t)r->heading;
    return true;
}

bool settings_save(const settings_t *s)
{
    record_t rec = {SETTINGS_MAGIC, SETTINGS_VERSION, (uint32_t)s->heading, 0};
    rec.crc = crc32(&rec, offsetof(record_t, crc));

    FLASH_EraseInitTypeDef erase = {0};
    uint32_t bad_sector = 0;
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = SETTINGS_SECTOR;
    erase.NbSectors = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;   /* VDD 2.7–3.6 V，可用 32 位元寫入 */

    const uint32_t *words = (const uint32_t *)&rec;
    HAL_FLASH_Unlock();
    bool ok = HAL_FLASHEx_Erase(&erase, &bad_sector) == HAL_OK;
    for (uint32_t i = 0; i < sizeof rec / 4U && ok; i++) {
        ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, SETTINGS_ADDR + 4U * i, words[i]) == HAL_OK;
    }
    HAL_FLASH_Lock();
    return ok && memcmp((const void *)SETTINGS_ADDR, &rec, sizeof rec) == 0;
}

const char *heading_name(heading_t h)
{
    return h == HEADING_OMNI_END_FRONT ? "omni end front" : "drive end front";
}
