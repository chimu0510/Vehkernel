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

/*
 * 四路正交編碼器，3.3 V 供電，皆為定時器編碼器模式（CH1/CH2）。
 *   A：TIM2（32 位元）PA15/PB3   B：TIM3 PA6/PA7
 *   C：TIM4 PB6/PB7              D：TIM5（32 位元）PA0/PA1
 * 底盤的兩顆馬達接在 A、B 兩路。
 */
#define ENC_A_TIM           TIM2
#define ENC_A1_PORT         GPIOA
#define ENC_A1_PIN          GPIO_PIN_15
#define ENC_A2_PORT         GPIOB
#define ENC_A2_PIN          GPIO_PIN_3
#define ENC_A_AF            GPIO_AF1_TIM2
#define ENC_B_TIM           TIM3
#define ENC_B1_PORT         GPIOA
#define ENC_B1_PIN          GPIO_PIN_6
#define ENC_B2_PORT         GPIOA
#define ENC_B2_PIN          GPIO_PIN_7
#define ENC_B_AF            GPIO_AF2_TIM3
#define ENC_C_TIM           TIM4
#define ENC_C1_PORT         GPIOB
#define ENC_C1_PIN          GPIO_PIN_6
#define ENC_C2_PORT         GPIOB
#define ENC_C2_PIN          GPIO_PIN_7
#define ENC_C_AF            GPIO_AF2_TIM4
#define ENC_D_TIM           TIM5
#define ENC_D1_PORT         GPIOA
#define ENC_D1_PIN          GPIO_PIN_0
#define ENC_D2_PORT         GPIOA
#define ENC_D2_PIN          GPIO_PIN_1
#define ENC_D_AF            GPIO_AF2_TIM5

/*
 * 馬達驅動接口 P2（2x5 排針，接一塊 D50A 的兩個通道），PWM 皆為 TIM8。
 *   馬達 A：PWM = PC9（TIM8_CH4），IN1 = PB13，IN2 = PB12
 *   馬達 B：PWM = PC8（TIM8_CH3），IN1 = PC0， IN2 = PE15
 * IN 腳板上有 10k 下拉：MCU 不驅動時 D50A 的 INA、INB 皆低，為脫機。
 * D50A：INA/INB = L/H 正轉、H/L 反轉、L/L 或 H/H 脫機；PWM 建議 18–60 kHz。
 * 接口 P1 的馬達 C、D（PC7/PC6）本專案未使用。
 */
#define MOTOR_PWM_TIM       TIM8
#define MOTOR_PWM_AF        GPIO_AF3_TIM8
#define MOTOR_A_PWM_PORT    GPIOC
#define MOTOR_A_PWM_PIN     GPIO_PIN_9
#define MOTOR_A_PWM_CH      TIM_CHANNEL_4
#define MOTOR_A_IN1_PORT    GPIOB
#define MOTOR_A_IN1_PIN     GPIO_PIN_13
#define MOTOR_A_IN2_PORT    GPIOB
#define MOTOR_A_IN2_PIN     GPIO_PIN_12
#define MOTOR_B_PWM_PORT    GPIOC
#define MOTOR_B_PWM_PIN     GPIO_PIN_8
#define MOTOR_B_PWM_CH      TIM_CHANNEL_3
#define MOTOR_B_IN1_PORT    GPIOC
#define MOTOR_B_IN1_PIN     GPIO_PIN_0
#define MOTOR_B_IN2_PORT    GPIOE
#define MOTOR_B_IN2_PIN     GPIO_PIN_15

/* 馬達 MD60P18：減速比 18；編碼器若與原廠相同為 500 線，四倍頻後輪子一圈 36000 計數。 */
#define MOTOR_GEAR_RATIO    18U
#define ENCODER_LINES       500U
#define COUNTS_PER_WHEEL_REV (ENCODER_LINES * 4U * MOTOR_GEAR_RATIO)

/* SBUS 接口 CN7：訊號經 NPN（Q3）反相後接 PD2 = UART5_RX，100000 baud 8E2。 */
#define SBUS_UART           UART5
#define SBUS_RX_PORT        GPIOD
#define SBUS_RX_PIN         GPIO_PIN_2
#define SBUS_AF             GPIO_AF8_UART5

/* CAN1：PD0 = RX、PD1 = TX，板載 VP230 收發器。 */
#define CAN_RX_PORT         GPIOD
#define CAN_RX_PIN          GPIO_PIN_0
#define CAN_TX_PORT         GPIOD
#define CAN_TX_PIN          GPIO_PIN_1
#define CAN_AF              GPIO_AF9_CAN1

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
