/*
 * 把控制板的濾波器（examples/common/ekf_ahrs.c、ahrs.c）編成主機的共用函式庫，
 * 供 native.py 以 ctypes 呼叫。濾波器原始碼直接 #include 進來，所以主機與控制板跑的是同一份代碼。
 * 輸出一律攤平成 float 陣列，欄位順序由 native.py 的 EKF_FIELDS / MAH_FIELDS 定義。
 */
#include <assert.h>
#include <stddef.h>
#include <string.h>

#include "ahrs.c"
#include "ekf_ahrs.c"
#include "frame.c"

static_assert(sizeof(ekf_ahrs_params_t) == 13 * sizeof(float), "ekf params must be 13 floats");
static_assert(sizeof(ahrs_params_t) == 3 * sizeof(float), "mahony params must be 3 floats");

size_t bench_ekf_size(void) { return sizeof(ekf_ahrs_t); }
size_t bench_mah_size(void) { return sizeof(ahrs_t); }

void bench_ekf_init(ekf_ahrs_t *f, const float *params, const float *accel)
{
    ekf_ahrs_params_t p;
    memcpy(&p, params, sizeof p);
    ekf_ahrs_init(f, &p, accel);
}

void bench_ekf_set_params(ekf_ahrs_t *f, const float *params)
{
    memcpy(&f->p, params, sizeof f->p);
}

void bench_ekf_step(ekf_ahrs_t *f, const float *gyro_dps, const float *accel_g, float dt)
{
    ekf_ahrs_step(f, gyro_dps, accel_g, dt);
}

void bench_ekf_set_state(ekf_ahrs_t *f, const float *q, const float *b)
{
    memcpy(f->q, q, sizeof f->q);
    memcpy(f->b, b, sizeof f->b);
}

static float *put(float *o, const float *src, size_t n)
{
    memcpy(o, src, n * sizeof(float));
    return o + n;
}

/* 欄位順序見 native.py 的 EKF_FIELDS。 */
void bench_ekf_export(const ekf_ahrs_t *f, float *o)
{
    const ekf_ahrs_diag_t *d = &f->diag;
    o = put(o, f->q, 4);
    o = put(o, f->b, 3);
    o = put(o, &f->P[0][0], 36);
    o = put(o, &d->P_prior[0][0], 36);
    o = put(o, d->v, 3);
    o = put(o, d->a_unit, 3);
    o = put(o, d->y, 3);
    o = put(o, &d->S[0][0], 9);
    o = put(o, &d->K[0][0], 18);
    o = put(o, d->dx, 6);
    *o++ = d->sigma_a;
    *o++ = d->nis;
    o = put(o, d->y_zupt, 3);
    *o++ = d->nis_zupt;
    *o++ = d->accel_used ? 1.0f : 0.0f;
    *o++ = d->zupt_used ? 1.0f : 0.0f;
    *o++ = f->still ? 1.0f : 0.0f;
}

/*
 * 整段資料一次跑完（離線模擬用，省掉逐筆呼叫的開銷）。
 * 每筆輸出 q(4) b(3) diag P(6) nis(1) flags(1)；flags bit0 still、bit1 accel_used、bit2 zupt_used。
 * 第 0 筆用來初始化（與韌體相同），輸出初始化後的狀態。
 */
void bench_ekf_run(const float *params, size_t n, const float *gyro, const float *accel, float dt, float *out)
{
    ekf_ahrs_t f;
    for (size_t k = 0; k < n; k++) {
        if (k == 0) {
            bench_ekf_init(&f, params, &accel[0]);
        } else {
            ekf_ahrs_step(&f, &gyro[3 * k], &accel[3 * k], dt);
        }
        float *o = &out[15 * k];
        o = put(o, f.q, 4);
        o = put(o, f.b, 3);
        for (int i = 0; i < 6; i++) {
            *o++ = f.P[i][i];
        }
        *o++ = k == 0 ? 0.0f : f.diag.nis;
        *o = (float)((f.still ? 1 : 0) | (f.diag.accel_used ? 2 : 0) | (f.diag.zupt_used ? 4 : 0));
    }
}

void bench_mah_init(ahrs_t *f, const float *params, const float *accel)
{
    ahrs_params_t p;
    memcpy(&p, params, sizeof p);
    ahrs_reset(f, &p, accel);
}

void bench_mah_set_params(ahrs_t *f, const float *params)
{
    memcpy(&f->p, params, sizeof f->p);
}

void bench_mah_update(ahrs_t *f, const float *gyro_dps, const float *accel_g, float dt)
{
    ahrs_update(f, gyro_dps, accel_g, dt, false);
}

void bench_mah_set_state(ahrs_t *f, const float *q, const float *b)
{
    if (q != NULL) {
        memcpy(f->q, q, sizeof f->q);
    }
    memcpy(f->b, b, sizeof f->b);
}

/* q(4) b(3) e(3) accel_used(1) */
void bench_mah_export(const ahrs_t *f, float *o)
{
    o = put(o, f->q, 4);
    o = put(o, f->b, 3);
    o = put(o, f->e, 3);
    *o = f->accel_used ? 1.0f : 0.0f;
}

/* 每筆輸出 q(4) b(3)；第 0 筆初始化；bias0 非 NULL 時初始化後把零偏設成它（rad/s）。 */
void bench_mah_run(const float *params, const float *bias0, size_t n, const float *gyro, const float *accel,
                 float dt, float *out)
{
    ahrs_t f;
    for (size_t k = 0; k < n; k++) {
        if (k == 0) {
            bench_mah_init(&f, params, &accel[0]);
            if (bias0 != NULL) {
                memcpy(f.b, bias0, sizeof f.b);
            }
        } else {
            ahrs_update(&f, &gyro[3 * k], &accel[3 * k], dt, false);
        }
        float *o = &out[7 * k];
        o = put(o, f.q, 4);
        put(o, f.b, 3);
    }
}

/* 協議測試用：與控制板相同的分幀。 */
size_t bench_frame_encode(const uint8_t *payload, size_t len, uint8_t *out) { return frame_encode(payload, len, out); }
int bench_frame_decode(uint8_t *buf, size_t len) { return frame_decode(buf, len); }
