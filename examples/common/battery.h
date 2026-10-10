/*
 * 電池電壓量測（PB0 = ADC1_IN8，分壓 11:1）。
 * 以內部參考電壓 VREFINT 與出廠校準值推算實際 VDDA，不假設 VDDA 剛好是 3.3 V。
 */
#ifndef BATTERY_H
#define BATTERY_H

#include <stdbool.h>

typedef struct {
    float vdda;          /* V */
    float vbat;          /* V */
    float vbat_noise;    /* 取樣標準差，V */
    float vrefint_raw;   /* ADC 計數平均 */
    float vbat_raw;      /* ADC 計數平均 */
} battery_reading_t;

void battery_init(void);

/* 每個通道取樣 64 次取平均；轉換失敗回傳 false。 */
bool battery_measure(battery_reading_t *out);

#endif /* BATTERY_H */
