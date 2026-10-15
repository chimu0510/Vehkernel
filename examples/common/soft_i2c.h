/*
 * 以 GPIO 開汲極輸出實作的 I2C 主機（約 250 kHz）。
 * C50C 的 IMU 腳位無法對應硬體 I2C，見 board.h。
 */
#ifndef SOFT_I2C_H
#define SOFT_I2C_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

typedef struct {
    GPIO_TypeDef *scl_port;
    uint16_t scl_pin;
    GPIO_TypeDef *sda_port;
    uint16_t sda_pin;
} soft_i2c_t;

/* 設定腳位並送 9 個時脈釋放可能卡住匯流排的從機。 */
void soft_i2c_init(const soft_i2c_t *bus);

/* 位址皆為 7 位元；任何一個位元組沒有收到 ACK 就回傳 false。 */
bool soft_i2c_probe(const soft_i2c_t *bus, uint8_t addr);
bool soft_i2c_write_reg(const soft_i2c_t *bus, uint8_t addr, uint8_t reg, uint8_t value);
bool soft_i2c_read_regs(const soft_i2c_t *bus, uint8_t addr, uint8_t reg, uint8_t *buf, size_t len);

#endif /* SOFT_I2C_H */
