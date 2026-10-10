#include "gyro_cal.h"

#include <math.h>
#include <string.h>

static float norm3(const float v[3])
{
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static double std_of(double sum, double sq, uint32_t n)
{
    const double mean = sum / n;
    return sqrt(fmax(sq / n - mean * mean, 0.0));
}

void gyro_cal_start(gyro_cal_t *c, float hz)
{
    memset(c, 0, sizeof *c);
    c->hz = hz;
    c->target_n = (uint32_t)(GYRO_CAL_SECONDS * hz);
}

static gyro_cal_state_t accumulate(gyro_cal_t *c, const imu_sample_t *s)
{
    const float an = norm3(s->accel_g);
    for (int k = 0; k < 3; k++) {
        c->sum_g[k] += (double)s->gyro_dps[k];
        c->sq_g[k] += (double)s->gyro_dps[k] * (double)s->gyro_dps[k];
        c->sum_a[k] += (double)s->accel_g[k];
    }
    c->sum_an += (double)an;
    c->sq_an += (double)an * (double)an;
    if (++c->n < c->target_n) {
        return GYRO_CAL_BUSY;
    }

    double worst = 0.0;
    for (int k = 0; k < 3; k++) {
        worst = fmax(worst, std_of(c->sum_g[k], c->sq_g[k], c->n));
    }
    c->last_std_dps = (float)worst;
    if (worst > (double)GYRO_CAL_MAX_STD_DPS || std_of(c->sum_an, c->sq_an, c->n) > (double)GYRO_CAL_MAX_STD_G) {
        const float hz = c->hz, last = c->last_std_dps;
        gyro_cal_start(c, hz);
        c->last_std_dps = last;
        return GYRO_CAL_RETRY;
    }
    for (int k = 0; k < 3; k++) {
        c->bias_dps[k] = (float)(c->sum_g[k] / c->n);
        c->level_g[k] = (float)(c->sum_a[k] / c->n);
    }
    c->calibrated = true;
    return GYRO_CAL_DONE;
}

gyro_cal_state_t gyro_cal_feed(gyro_cal_t *c, const imu_sample_t *s, float out_dps[3])
{
    if (!c->calibrated) {
        out_dps[0] = out_dps[1] = out_dps[2] = 0.0f;
        return accumulate(c, s);
    }

    bool quiet = fabsf(norm3(s->accel_g) - 1.0f) < GYRO_STILL_G;
    for (int k = 0; k < 3; k++) {
        out_dps[k] = s->gyro_dps[k] - c->bias_dps[k];
        quiet &= fabsf(out_dps[k]) < GYRO_STILL_DPS;
    }
    c->still_n = quiet ? c->still_n + 1U : 0U;
    c->still_s = (float)c->still_n / c->hz;
    c->still = c->still_s >= GYRO_STILL_SECONDS;

    if (c->still) {
        const float alpha = 1.0f / (GYRO_TRACK_TAU_S * c->hz);
        for (int k = 0; k < 3; k++) {
            c->bias_dps[k] += alpha * (s->gyro_dps[k] - c->bias_dps[k]);
            out_dps[k] = s->gyro_dps[k] - c->bias_dps[k];
        }
        c->bias_updates++;
    }
    return GYRO_CAL_TRACKING;
}
