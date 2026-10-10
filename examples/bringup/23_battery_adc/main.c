/*
 * 23 電池電壓 ADC（PB0 = ADC1_IN8，分壓 11:1）。
 * 量測方式見 common/battery.h。需要接上電池；只接 USB 供電時會判定失敗。
 * 可選：輸入萬用電表量到的電池電壓，檢查誤差是否在 3% 內。
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "battery.h"
#include "board.h"
#include "console.h"
#include "report.h"

#define METER_TIMEOUT_MS   30000U
#define METER_TOLERANCE    0.03f

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
    battery_reading_t r;

    board_init();
    report_begin("23_battery_adc");
    battery_init();

    if (!report_check(battery_measure(&r), "adc conversion", "64 samples per channel")) {
        report_end();
        report_halt();
    }

    report_check(r.vdda > 3.0f && r.vdda < 3.6f, "vdda", "%.3f V (VREFINT raw %.1f, cal %u)",
                 (double)r.vdda, (double)r.vrefint_raw, *VREFINT_CAL_ADDR);
    report_check(r.vbat > 6.0f && r.vbat < 30.0f, "battery voltage",
                 "%.2f V (raw %.1f); below 6 V usually means only USB power is connected",
                 (double)r.vbat, (double)r.vbat_raw);
    report_check(r.vbat_noise < 0.05f, "battery noise", "std %.1f mV (expect < 50 mV)",
                 (double)(r.vbat_noise * 1000.0f));
    test_meter(r.vbat);

    report_end();

    printf("  streaming battery voltage at 1 Hz\n");
    for (;;) {
        if (battery_measure(&r)) {
            printf("  vbat %.2f V\n", (double)r.vbat);
        }
        HAL_Delay(1000);
    }
}
