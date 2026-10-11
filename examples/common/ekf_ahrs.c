#include "ekf_ahrs.h"

#include <math.h>
#include <string.h>

#define N        6
#define DEG2RAD  (3.14159265f / 180.0f)
#define RAD2DEG  (180.0f / 3.14159265f)
#define STILL_ALPHA 0.05f    /* 靜止偵測的移動平均係數，200 Hz 下約 0.1 s */

static void normalize_q(float q[4])
{
    const float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    for (int i = 0; i < 4; i++) {
        q[i] /= n;
    }
}

/* q ← q ⊗ [1, θ/2]（小角度，本體座標），再正規化。 */
static void rotate_q(float q[4], const float th[3])
{
    const float hx = 0.5f * th[0], hy = 0.5f * th[1], hz = 0.5f * th[2];
    const float a0 = q[0], a1 = q[1], a2 = q[2], a3 = q[3];
    q[0] = a0 - a1 * hx - a2 * hy - a3 * hz;
    q[1] = a1 + a0 * hx + a2 * hz - a3 * hy;
    q[2] = a2 + a0 * hy - a1 * hz + a3 * hx;
    q[3] = a3 + a0 * hz + a1 * hy - a2 * hx;
    normalize_q(q);
}

/* 本體座標下的重力方向（世界 z 軸在本體座標的分量）。 */
static void gravity_body(const float q[4], float v[3])
{
    v[0] = 2.0f * (q[1] * q[3] - q[0] * q[2]);
    v[1] = 2.0f * (q[0] * q[1] + q[2] * q[3]);
    v[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
}

static void symmetrize(float P[N][N])
{
    for (int i = 0; i < N; i++) {
        for (int j = i + 1; j < N; j++) {
            const float m = 0.5f * (P[i][j] + P[j][i]);
            P[i][j] = P[j][i] = m;
        }
    }
}

void ekf_ahrs_init(ekf_ahrs_t *f, const ekf_ahrs_params_t *p, const float a[3])
{
    static const ekf_ahrs_params_t defaults = EKF_AHRS_DEFAULT_PARAMS;
    const ekf_ahrs_params_t params = p != NULL ? *p : defaults;

    memset(f, 0, sizeof *f);
    f->p = params;

    const float roll = atan2f(a[1], a[2]);
    const float pitch = atan2f(-a[0], sqrtf(a[1] * a[1] + a[2] * a[2]));
    const float cr = cosf(roll / 2.0f), sr = sinf(roll / 2.0f);
    const float cp = cosf(pitch / 2.0f), sp = sinf(pitch / 2.0f);
    f->q[0] = cr * cp;
    f->q[1] = sr * cp;
    f->q[2] = cr * sp;
    f->q[3] = -sr * sp;
    normalize_q(f->q);

    const float sa = f->p.init_angle_deg * DEG2RAD, sb = f->p.init_bias_dps * DEG2RAD;
    f->P[0][0] = f->P[1][1] = sa * sa;
    f->P[2][2] = 1e-6f;                 /* yaw 定義為 0，起點沒有不確定度 */
    f->P[3][3] = f->P[4][4] = f->P[5][5] = sb * sb;
}

/* P ← F P Fᵀ + Q，F = I + A·dt，A = [[-[ω]×, -I], [0, 0]]。 */
static void predict(ekf_ahrs_t *f, const float w[3], float dt)
{
    float F[N][N] = {{0}}, FP[N][N], Pn[N][N];

    for (int i = 0; i < N; i++) {
        F[i][i] = 1.0f;
    }
    /* -[ω]× dt */
    F[0][1] = w[2] * dt;
    F[0][2] = -w[1] * dt;
    F[1][0] = -w[2] * dt;
    F[1][2] = w[0] * dt;
    F[2][0] = w[1] * dt;
    F[2][1] = -w[0] * dt;
    F[0][3] = F[1][4] = F[2][5] = -dt;

    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            float s = 0.0f;
            for (int k = 0; k < N; k++) {
                s += F[i][k] * f->P[k][j];
            }
            FP[i][j] = s;
        }
    }
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            float s = 0.0f;
            for (int k = 0; k < N; k++) {
                s += FP[i][k] * F[j][k];
            }
            Pn[i][j] = s;
        }
    }
    const float qa = f->p.gyro_noise_dps * DEG2RAD, qb = f->p.bias_walk_dps * DEG2RAD;
    for (int i = 0; i < 3; i++) {
        Pn[i][i] += qa * qa * dt;
        Pn[3 + i][3 + i] += qb * qb * dt;
    }
    memcpy(f->P, Pn, sizeof Pn);
    symmetrize(f->P);
}

static bool invert3(const float m[3][3], float inv[3][3])
{
    const float c00 = m[1][1] * m[2][2] - m[1][2] * m[2][1];
    const float c01 = m[1][2] * m[2][0] - m[1][0] * m[2][2];
    const float c02 = m[1][0] * m[2][1] - m[1][1] * m[2][0];
    const float det = m[0][0] * c00 + m[0][1] * c01 + m[0][2] * c02;
    if (fabsf(det) < 1e-20f) {
        return false;
    }
    const float d = 1.0f / det;
    inv[0][0] = c00 * d;
    inv[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * d;
    inv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * d;
    inv[1][0] = c01 * d;
    inv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * d;
    inv[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * d;
    inv[2][0] = c02 * d;
    inv[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * d;
    inv[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * d;
    return true;
}

/*
 * 三維量測的 EKF 修正：H 為 3×6，y 為創新量，r 為各軸量測變異數；把誤差併回名義狀態。
 * 回傳 NIS（yᵀ S⁻¹ y）；S 不可逆時不修正並回傳負值。S、K、dx 可為 NULL。
 */
static float update3(ekf_ahrs_t *f, const float H[3][N], const float y[3], float r,
                     float S_out[3][3], float K_out[N][3], float dx_out[N])
{
    float PHt[N][3], S[3][3], Si[3][3], K[N][3], dx[N];

    for (int i = 0; i < N; i++) {
        for (int j = 0; j < 3; j++) {
            float s = 0.0f;
            for (int k = 0; k < N; k++) {
                s += f->P[i][k] * H[j][k];
            }
            PHt[i][j] = s;
        }
    }
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            float s = i == j ? r : 0.0f;
            for (int k = 0; k < N; k++) {
                s += H[i][k] * PHt[k][j];
            }
            S[i][j] = s;
        }
    }
    if (!invert3(S, Si)) {
        return -1.0f;
    }
    float nis = 0.0f;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            nis += y[i] * Si[i][j] * y[j];
        }
    }
    for (int i = 0; i < N; i++) {
        dx[i] = 0.0f;
        for (int j = 0; j < 3; j++) {
            float s = 0.0f;
            for (int k = 0; k < 3; k++) {
                s += PHt[i][k] * Si[k][j];
            }
            K[i][j] = s;
            dx[i] += s * y[j];
        }
    }
    /* P ← P − K (P Hᵀ)ᵀ */
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            f->P[i][j] -= K[i][0] * PHt[j][0] + K[i][1] * PHt[j][1] + K[i][2] * PHt[j][2];
        }
    }
    symmetrize(f->P);

    rotate_q(f->q, dx);
    for (int i = 0; i < 3; i++) {
        f->b[i] += dx[3 + i];
    }
    if (S_out != NULL) {
        memcpy(S_out, S, sizeof S);
    }
    if (K_out != NULL) {
        memcpy(K_out, K, sizeof K);
    }
    if (dx_out != NULL) {
        memcpy(dx_out, dx, sizeof dx);
    }
    return nis;
}

static void update_accel(ekf_ahrs_t *f, const float a[3])
{
    ekf_ahrs_diag_t *d = &f->diag;
    const float an = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    const float dev = fabsf(an - 1.0f);
    float v[3];

    gravity_body(f->q, v);
    memcpy(d->v, v, sizeof v);
    d->accel_used = false;
    if (dev > f->p.accel_reject_g || an < 1e-3f) {
        f->accel_rejects++;
        return;
    }
    /* 姿態誤差 δθ 使預測方向變成 v + [v]× δθ，所以 H_θ = [v]×。 */
    const float H[3][N] = {
        {0.0f, -v[2], v[1], 0, 0, 0},
        {v[2], 0.0f, -v[0], 0, 0, 0},
        {-v[1], v[0], 0.0f, 0, 0, 0},
    };
    const float y[3] = {a[0] / an - v[0], a[1] / an - v[1], a[2] / an - v[2]};
    const float sigma = f->p.accel_noise + f->p.accel_adapt * dev;
    for (int i = 0; i < 3; i++) {
        d->a_unit[i] = a[i] / an;
        d->y[i] = y[i];
    }
    d->sigma_a = sigma;
    d->nis = update3(f, H, y, sigma * sigma, d->S, d->K, d->dx);
    d->accel_used = d->nis >= 0.0f;
    f->accel_updates++;
}

static void update_zupt(ekf_ahrs_t *f, const float w_meas[3])
{
    /* 靜止時量測值 = 零偏：y = ω_量測 − b，H = [0 I]。 */
    const float H[3][N] = {
        {0, 0, 0, 1, 0, 0},
        {0, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0, 1},
    };
    const float y[3] = {w_meas[0] - f->b[0], w_meas[1] - f->b[1], w_meas[2] - f->b[2]};
    const float r = f->p.zupt_noise_dps * DEG2RAD;
    memcpy(f->diag.y_zupt, y, sizeof y);
    f->diag.nis_zupt = update3(f, H, y, r * r, NULL, NULL, NULL);
    f->diag.zupt_used = true;
    f->zupt_updates++;
}

static void detect_still(ekf_ahrs_t *f, const float w[3], const float a[3], float dt)
{
    const float lim_var = f->p.still_var_dps * DEG2RAD, lim_w = f->p.still_dps * DEG2RAD;
    const float an = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    bool quiet = fabsf(an - 1.0f) < f->p.still_g;

    for (int i = 0; i < 3; i++) {
        const float d = w[i] - f->mean[i];
        f->mean[i] += STILL_ALPHA * d;
        f->var[i] = (1.0f - STILL_ALPHA) * (f->var[i] + STILL_ALPHA * d * d);
        quiet &= f->var[i] < lim_var * lim_var && fabsf(w[i] - f->b[i]) < lim_w;
    }
    f->quiet_n = quiet ? f->quiet_n + 1U : 0U;
    f->still = (float)f->quiet_n * dt >= f->p.still_s;
}

void ekf_ahrs_step(ekf_ahrs_t *f, const float gyro_dps[3], const float accel_g[3], float dt)
{
    const float wm[3] = {gyro_dps[0] * DEG2RAD, gyro_dps[1] * DEG2RAD, gyro_dps[2] * DEG2RAD};
    const float w[3] = {wm[0] - f->b[0], wm[1] - f->b[1], wm[2] - f->b[2]};
    const float th[3] = {w[0] * dt, w[1] * dt, w[2] * dt};

    rotate_q(f->q, th);
    predict(f, w, dt);
    memcpy(f->diag.P_prior, f->P, sizeof f->P);
    update_accel(f, accel_g);
    detect_still(f, wm, accel_g, dt);
    f->diag.zupt_used = false;
    if (f->still && f->p.zupt_enable != 0.0f) {
        update_zupt(f, wm);
    }
}

void ekf_ahrs_euler(const ekf_ahrs_t *f, float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    const float *q = f->q;
    const float sp = fmaxf(fminf(2.0f * (q[0] * q[2] - q[3] * q[1]), 1.0f), -1.0f);

    *roll_deg = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2])) * RAD2DEG;
    *pitch_deg = asinf(sp) * RAD2DEG;
    *yaw_deg = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3])) * RAD2DEG;
}

void ekf_ahrs_bias_dps(const ekf_ahrs_t *f, float bias_dps[3])
{
    for (int i = 0; i < 3; i++) {
        bias_dps[i] = f->b[i] * RAD2DEG;
    }
}

void ekf_ahrs_sigma(const ekf_ahrs_t *f, float angle_deg[3], float bias_dps[3])
{
    for (int i = 0; i < 3; i++) {
        angle_deg[i] = sqrtf(fmaxf(f->P[i][i], 0.0f)) * RAD2DEG;
        bias_dps[i] = sqrtf(fmaxf(f->P[3 + i][3 + i], 0.0f)) * RAD2DEG;
    }
}
