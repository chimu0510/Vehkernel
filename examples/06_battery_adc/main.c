/*
 * 06 電池電壓 ADC（PB0 = ADC1_IN8，分壓 11:1）。
 * 以內部參考電壓 VREFINT 與出廠校準值推算實際 VDDA，再換算電池電壓，
 * 不假設 VDDA 剛好是 3.3 V。需要接上電池；只接 USB 供電時會判定失敗。
 * 可選：輸入萬用電表量到的電池電壓，檢查誤差是否在 3% 內。
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "board.h"
#include "console.h"
#include "report.h"

#define VREFINT_CAL_VDDA   3.3f   /* VREFINT_CAL_ADDR（HAL 定義）的校準值是在 VDDA = 3.3 V 時量得 */
#define ADC_FULL_SCALE     4095.0f
#define SAMPLES            64U
#define METER_TIMEOUT_MS   30000U
#define METER_TOLERANCE    0.03f

static ADC_HandleTypeDef s_adc;

static void adc_init(void)
{
    GPIO_InitTypeDef io = {0};

    __HAL_RCC_ADC1_CLK_ENABLE();
    io.Mode = GPIO_MODE_ANALOG;
    io.Pull = GPIO_NOPULL;
    io.Pin = VBAT_PIN;
    HAL_GPIO_Init(VBAT_PORT, &io);

    s_adc.Instance = ADC1;
    s_adc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;   /* 84 / 4 = 21 MHz */
    s_adc.Init.Resolution = ADC_RESOLUTION_12B;
    s_adc.Init.ScanConvMode = DISABLE;
    s_adc.Init.ContinuousConvMode = DISABLE;
    s_adc.Init.DiscontinuousConvMode = DISABLE;
    s_adc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    s_adc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    s_adc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    s_adc.Init.NbrOfConversion = 1;
    s_adc.Init.DMAContinuousRequests = DISABLE;
    s_adc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    HAL_ADC_Init(&s_adc);
}

/* 對單一通道取樣 SAMPLES 次，回傳平均與標準差（ADC 計數）。 */
static bool adc_measure(uint32_t channel, float *mean, float *std)
{
    ADC_ChannelConfTypeDef ch = {0};
    float sum = 0.0f, sq = 0.0f;

    ch.Channel = channel;
    ch.Rank = 1;
    ch.SamplingTime = ADC_SAMPLETIME_480CYCLES; /* 分壓電阻阻抗高，取樣時間拉長 */
    if (HAL_ADC_ConfigChannel(&s_adc, &ch) != HAL_OK) {
        return false;
    }
    for (unsigned i = 0; i < SAMPLES; i++) {
        HAL_ADC_Start(&s_adc);
        if (HAL_ADC_PollForConversion(&s_adc, 10) != HAL_OK) {
            return false;
        }
        const float v = (float)HAL_ADC_GetValue(&s_adc);
        sum += v;
        sq += v * v;
    }
    *mean = sum / (float)SAMPLES;
    *std = sqrtf(fmaxf(sq / (float)SAMPLES - *mean * *mean, 0.0f));
    return true;
}

static void test_meter(float vbat)
{
    char line[16];

    printf("  optional: measure the battery with a multimeter and type the voltage (e.g. 24.6),\n"
           "  or just press Enter to skip (%lus)\n> ", (unsigned long)(METER_TIMEOUT_MS / 1000U));
    console_flush_input();
    if (console_readline(line, sizeof line, METER_TIMEOUT_MS) <= 0) {
        printf("\n");
        report_skip("vs multimeter", "no value entered");
        return;
    }
    const float meter = strtof(line, NULL);
    if (meter <= 0.0f) {
        report_skip("vs multimeter", "could not parse \"%s\"", line);
        return;
    }
    const float err = (vbat - meter) / meter;
    report_check(fabsf(err) <= METER_TOLERANCE, "vs multimeter",
                 "adc %.2f V, meter %.2f V, error %+.1f%% (expect within 3%%)",
                 (double)vbat, (double)meter, (double)(err * 100.0f));
}

int main(void)
{
    float ref_mean, ref_std, bat_mean, bat_std;

    board_init();
    report_begin("06_battery_adc");
    adc_init();
    ADC->CCR |= ADC_CCR_TSVREFE; /* 開啟 VREFINT 通道 */
    HAL_Delay(1);

    if (!report_check(adc_measure(ADC_CHANNEL_VREFINT, &ref_mean, &ref_std) &&
                      adc_measure(VBAT_ADC_CHANNEL, &bat_mean, &bat_std),
                      "adc conversion", "%u samples per channel", SAMPLES)) {
        report_end();
        report_halt();
    }

    const float vdda = VREFINT_CAL_VDDA * (float)*VREFINT_CAL_ADDR / ref_mean;
    const float volts_per_count = vdda / ADC_FULL_SCALE * VBAT_DIVIDER;
    const float vbat = bat_mean * volts_per_count;
    const float vbat_noise = bat_std * volts_per_count;

    report_check(vdda > 3.0f && vdda < 3.6f, "vdda", "%.3f V (VREFINT raw %.1f, cal %u)",
                 (double)vdda, (double)ref_mean, *VREFINT_CAL_ADDR);
    report_check(vbat > 6.0f && vbat < 30.0f, "battery voltage",
                 "%.2f V (raw %.1f); below 6 V usually means only USB power is connected",
                 (double)vbat, (double)bat_mean);
    report_check(vbat_noise < 0.05f, "battery noise", "std %.1f mV (expect < 50 mV)",
                 (double)(vbat_noise * 1000.0f));
    test_meter(vbat);

    report_end();

    printf("  streaming battery voltage at 1 Hz\n");
    for (;;) {
        if (adc_measure(VBAT_ADC_CHANNEL, &bat_mean, &bat_std)) {
            printf("  vbat %.2f V\n", (double)(bat_mean * volts_per_count));
        }
        HAL_Delay(1000);
    }
}
