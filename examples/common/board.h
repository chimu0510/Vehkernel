/*
 * C50C 控制板腳位與板級常數。
 *
 * 來源：原廠資料《C50C-V4.0V4.1V4.2 集成板资源分配》與原廠代碼的
 * HARDWARE/Inc 標頭檔；兩者不一致時以原廠代碼為準（原廠代碼實際驅動過板子）。
 */
#ifndef BOARD_H
#define BOARD_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

/* 系統時脈：外部 8 MHz 晶振，PLL 到 168 MHz（USB 48 MHz）。 */
#define BOARD_HSE_HZ        8000000U
#define BOARD_SYSCLK_HZ     168000000U

/*
 * RGB LED（同時接到 RGB 燈帶接口），高電位點亮。舊版 V1.0 的 LED 在別的腳位且低電位點亮。
 * 紅、綠兩支腳以 02_led 在 V1.1 實機確認：PE6 = 紅、PB15 = 綠，與原廠代碼的命名
 * （LED_R = PB15、LED_G = PE6）相反。
 */
#define LED_R_PORT          GPIOE
#define LED_R_PIN           GPIO_PIN_6
#define LED_G_PORT          GPIOB
#define LED_G_PIN           GPIO_PIN_15
#define LED_B_PORT          GPIOB
#define LED_B_PIN           GPIO_PIN_14
#define LED_ON_LEVEL        GPIO_PIN_SET

/* 蜂鳴器，高電位發聲；需插上板上的蜂鳴器跳線帽。 */
#define BUZZER_PORT         GPIOD
#define BUZZER_PIN          GPIO_PIN_11

/* 使用者按鍵，上拉，按下為低電位。 */
#define USER_KEY_PORT       GPIOC
#define USER_KEY_PIN        GPIO_PIN_5

/* 使能開關（急停），上拉；原廠代碼以高電位視為允許馬達運轉。 */
#define ENABLE_SW_PORT      GPIOE
#define ENABLE_SW_PIN       GPIO_PIN_4

/* 硬體版本腳，下拉輸入：bit0 = PE2、bit1 = PE3、bit2 = PC13。 */
#define VER_BIT0_PORT       GPIOE
#define VER_BIT0_PIN        GPIO_PIN_2
#define VER_BIT1_PORT       GPIOE
#define VER_BIT1_PIN        GPIO_PIN_3
#define VER_BIT2_PORT       GPIOC
#define VER_BIT2_PIN        GPIO_PIN_13

/* USART1：Type-C 串口 1（CH9102F），也是串口燒錄口。範例的主控台。 */
#define CONSOLE_UART        USART1
#define CONSOLE_TX_PORT     GPIOA
#define CONSOLE_TX_PIN      GPIO_PIN_9
#define CONSOLE_RX_PORT     GPIOA
#define CONSOLE_RX_PIN      GPIO_PIN_10
#define CONSOLE_AF          GPIO_AF7_USART1
#define CONSOLE_BAUD        115200U

/* USART3：Type-C 串口 3（第二顆 CH9102F）。 */
#define AUX_UART            USART3
#define AUX_TX_PORT         GPIOB
#define AUX_TX_PIN          GPIO_PIN_10
#define AUX_RX_PORT         GPIOB
#define AUX_RX_PIN          GPIO_PIN_11
#define AUX_AF              GPIO_AF7_USART3

/*
 * ICM-20948 的 I2C。SCL = PB9、SDA = PB8，與 STM32F407 硬體 I2C1 的
 * 腳位定義（PB8 = SCL、PB9 = SDA）相反，所以只能用軟體 I2C。
 */
#define IMU_SCL_PORT        GPIOB
#define IMU_SCL_PIN         GPIO_PIN_9
#define IMU_SDA_PORT        GPIOB
#define IMU_SDA_PIN         GPIO_PIN_8
#define ICM20948_I2C_ADDR   0x68U   /* AD0 = 0 */
#define MPU6050_I2C_ADDR    0x68U   /* 舊版 C50C（V1.0）使用 */

/* 電池電壓：PB0 = ADC1_IN8，分壓比 11:1。 */
#define VBAT_PORT           GPIOB
#define VBAT_PIN            GPIO_PIN_0
#define VBAT_ADC_CHANNEL    ADC_CHANNEL_8
#define VBAT_DIVIDER        11.0f

/* 板級初始化：HAL、168 MHz 時脈、GPIO 時脈、DWT 計時器、主控台。 */
void board_init(void);

/* 依 board_init() 結果回報外部晶振是否啟動成功。 */
bool board_hse_ok(void);

/* 以 DWT 週期計數器實作的忙等延遲與時間戳。 */
void delay_us(uint32_t us);
uint32_t micros(void);

void led_set(bool r, bool g, bool b);
void buzzer_set(bool on);
bool user_key_pressed(void);
bool enable_switch_high(void);

#endif /* BOARD_H */
