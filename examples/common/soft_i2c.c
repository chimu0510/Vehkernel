#include "soft_i2c.h"

#include "board.h"

#define HALF_PERIOD_US 2U

static void scl(const soft_i2c_t *bus, bool high)
{
    HAL_GPIO_WritePin(bus->scl_port, bus->scl_pin, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void sda(const soft_i2c_t *bus, bool high)
{
    HAL_GPIO_WritePin(bus->sda_port, bus->sda_pin, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static bool sda_read(const soft_i2c_t *bus)
{
    return HAL_GPIO_ReadPin(bus->sda_port, bus->sda_pin) == GPIO_PIN_SET;
}

/* 拉高 SCL 後等從機放開（clock stretching），最多 1 ms。 */
static void scl_release(const soft_i2c_t *bus)
{
    const uint32_t start = micros();
    scl(bus, true);
    while (HAL_GPIO_ReadPin(bus->scl_port, bus->scl_pin) == GPIO_PIN_RESET &&
           micros() - start < 1000U) {
    }
}

static void start(const soft_i2c_t *bus)
{
    sda(bus, true);
    scl_release(bus);
    delay_us(HALF_PERIOD_US);
    sda(bus, false);
    delay_us(HALF_PERIOD_US);
    scl(bus, false);
}

static void stop(const soft_i2c_t *bus)
{
    sda(bus, false);
    delay_us(HALF_PERIOD_US);
    scl_release(bus);
    delay_us(HALF_PERIOD_US);
    sda(bus, true);
    delay_us(HALF_PERIOD_US);
}

/* 送出一個位元組，回傳是否收到 ACK。 */
static bool write_byte(const soft_i2c_t *bus, uint8_t byte)
{
    for (int i = 7; i >= 0; i--) {
        sda(bus, (byte >> i) & 1U);
        delay_us(HALF_PERIOD_US);
        scl_release(bus);
        delay_us(HALF_PERIOD_US);
        scl(bus, false);
    }
    sda(bus, true);
    delay_us(HALF_PERIOD_US);
    scl_release(bus);
    const bool ack = !sda_read(bus);
    delay_us(HALF_PERIOD_US);
    scl(bus, false);
    return ack;
}

static uint8_t read_byte(const soft_i2c_t *bus, bool ack)
{
    uint8_t byte = 0;

    sda(bus, true);
    for (int i = 0; i < 8; i++) {
        delay_us(HALF_PERIOD_US);
        scl_release(bus);
        byte = (uint8_t)((byte << 1) | (sda_read(bus) ? 1U : 0U));
        delay_us(HALF_PERIOD_US);
        scl(bus, false);
    }
    sda(bus, !ack);
    delay_us(HALF_PERIOD_US);
    scl_release(bus);
    delay_us(HALF_PERIOD_US);
    scl(bus, false);
    sda(bus, true);
    return byte;
}

void soft_i2c_init(const soft_i2c_t *bus)
{
    GPIO_InitTypeDef io = {0};

    io.Mode = GPIO_MODE_OUTPUT_OD;
    io.Pull = GPIO_PULLUP;
    io.Speed = GPIO_SPEED_FREQ_HIGH;
    io.Pin = bus->scl_pin;
    HAL_GPIO_Init(bus->scl_port, &io);
    io.Pin = bus->sda_pin;
    HAL_GPIO_Init(bus->sda_port, &io);

    sda(bus, true);
    for (int i = 0; i < 9; i++) {
        scl(bus, false);
        delay_us(HALF_PERIOD_US);
        scl_release(bus);
        delay_us(HALF_PERIOD_US);
    }
    stop(bus);
}

bool soft_i2c_probe(const soft_i2c_t *bus, uint8_t addr)
{
    start(bus);
    const bool ack = write_byte(bus, (uint8_t)(addr << 1));
    stop(bus);
    return ack;
}

bool soft_i2c_write_reg(const soft_i2c_t *bus, uint8_t addr, uint8_t reg, uint8_t value)
{
    start(bus);
    const bool ok = write_byte(bus, (uint8_t)(addr << 1)) &&
                    write_byte(bus, reg) &&
                    write_byte(bus, value);
    stop(bus);
    return ok;
}

bool soft_i2c_read_regs(const soft_i2c_t *bus, uint8_t addr, uint8_t reg, uint8_t *buf, size_t len)
{
    start(bus);
    if (!write_byte(bus, (uint8_t)(addr << 1)) || !write_byte(bus, reg)) {
        stop(bus);
        return false;
    }
    start(bus); /* repeated start */
    if (!write_byte(bus, (uint8_t)((addr << 1) | 1U))) {
        stop(bus);
        return false;
    }
    for (size_t i = 0; i < len; i++) {
        buf[i] = read_byte(bus, i + 1 < len);
    }
    stop(bus);
    return true;
}
