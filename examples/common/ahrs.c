#include "ahrs.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define DEG2RAD (3.14159265f / 180.0f)
#define RAD2DEG (180.0f / 3.14159265f)

static void normalize(float q[4])
{
    const float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    for (int i = 0; i < 4; i++) {
        q[i] /= n;
    }
}

void ahrs_reset(ahrs_t *f, const ahrs_params_t *p, const float a[3])
{
    static const ahrs_params_t defaults = AHRS_DEFAULT_PARAMS;
    const float roll = atan2f(a[1], a[2]);
    const float pitch = atan2f(-a[0], sqrtf(a[1] * a[1] + a[2] * a[2]));
    const float cr = cosf(roll / 2.0f), sr = sinf(roll / 2.0f);
    const float cp = cosf(pitch / 2.0f), sp = sinf(pitch / 2.0f);

    memset(f, 0, sizeof *f);
    f->p = p != NULL ? *p : defaults;
    /* yaw = 0 時的 ZYX 尤拉角轉四元數。 */
    f->q[0] = cr * cp;
    f->q[1] = sr * cp;
    f->q[2] = cr * sp;
    f->q[3] = -sr * sp;
    normalize(f->q);
}

void ahrs_update(ahrs_t *f, const float gyro_dps[3], const float accel_g[3], float dt, bool hold)
{
    float *q = f->q;
    float g[3];

    for (int i = 0; i < 3; i++) {
        g[i] = hold ? 0.0f : gyro_dps[i] * DEG2RAD - f->b[i];
    }

    float ax = accel_g[0], ay = accel_g[1], az = accel_g[2];
    const float an = sqrtf(ax * ax + ay * ay + az * az);
    f->accel_used = an > 1e-3f && fabsf(an - 1.0f) < f->p.accel_gate_g;
    if (f->accel_used) {
        ax /= an;
        ay /= an;
        az /= an;
        /* 由目前姿態估計的重力方向（本體座標），與量測方向的外積就是姿態誤差。 */
        const float vx = 2.0f * (q[1] * q[3] - q[0] * q[2]);
        const float vy = 2.0f * (q[0] * q[1] + q[2] * q[3]);
        const float vz = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
        f->e[0] = ay * vz - az * vy;
        f->e[1] = az * vx - ax * vz;
        f->e[2] = ax * vy - ay * vx;
        for (int i = 0; i < 3; i++) {
            if (f->p.ki > 0.0f && !hold) {
                f->b[i] -= f->p.ki * f->e[i] * dt;
            }
            g[i] += f->p.kp * f->e[i];
        }
    } else {
        f->e[0] = f->e[1] = f->e[2] = 0.0f;
    }

    /* q̇ = ½ q ⊗ (0, ω) */
    const float h = 0.5f * dt;
    const float a0 = q[0], a1 = q[1], a2 = q[2], a3 = q[3];
    q[0] += h * (-a1 * g[0] - a2 * g[1] - a3 * g[2]);
    q[1] += h * (a0 * g[0] + a2 * g[2] - a3 * g[1]);
    q[2] += h * (a0 * g[1] - a1 * g[2] + a3 * g[0]);
    q[3] += h * (a0 * g[2] + a1 * g[1] - a2 * g[0]);
    normalize(q);
}

void ahrs_euler(const ahrs_t *f, float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    const float *q = f->q;
    const float sp = fmaxf(fminf(2.0f * (q[0] * q[2] - q[3] * q[1]), 1.0f), -1.0f);

    *roll_deg = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2])) * RAD2DEG;
    *pitch_deg = asinf(sp) * RAD2DEG;
    *yaw_deg = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3])) * RAD2DEG;
}
