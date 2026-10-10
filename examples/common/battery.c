#include "battery.h"

#include <math.h>

#include "board.h"

#define VREFINT_CAL_VDDA   3.3f   /* VREFINT_CAL_ADDR（HAL 定義）的校準值是在 VDDA = 3.3 V 時量得 */
#define ADC_FULL_SCALE     4095.0f
#define SAMPLES            64U

static ADC_HandleTypeDef s_adc;

void battery_init(void)
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

    ADC->CCR |= ADC_CCR_TSVREFE; /* 開啟 VREFINT 通道 */
    HAL_Delay(1);
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

bool battery_measure(battery_reading_t *out)
{
    float ref_std, bat_std;

    if (!adc_measure(ADC_CHANNEL_VREFINT, &out->vrefint_raw, &ref_std) ||
        !adc_measure(VBAT_ADC_CHANNEL, &out->vbat_raw, &bat_std)) {
        return false;
    }
    out->vdda = VREFINT_CAL_VDDA * (float)*VREFINT_CAL_ADDR / out->vrefint_raw;
    const float volts_per_count = out->vdda / ADC_FULL_SCALE * VBAT_DIVIDER;
    out->vbat = out->vbat_raw * volts_per_count;
    out->vbat_noise = bat_std * volts_per_count;
    return true;
}
