#include "ahrs.h"

#include <math.h>

#define DEG2RAD (3.14159265f / 180.0f)
#define RAD2DEG (180.0f / 3.14159265f)

static float q0 = 1.0f, q1, q2, q3;    /* 本體到世界座標的四元數 */
static bool s_accel_used;

static void normalize(void)
{
    const float n = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 /= n;
    q1 /= n;
    q2 /= n;
    q3 /= n;
}

void ahrs_reset(const float a[3])
{
    const float roll = atan2f(a[1], a[2]);
    const float pitch = atan2f(-a[0], sqrtf(a[1] * a[1] + a[2] * a[2]));
    const float cr = cosf(roll / 2.0f), sr = sinf(roll / 2.0f);
    const float cp = cosf(pitch / 2.0f), sp = sinf(pitch / 2.0f);

    /* yaw = 0 時的 ZYX 尤拉角轉四元數。 */
    q0 = cr * cp;
    q1 = sr * cp;
    q2 = cr * sp;
    q3 = -sr * sp;
    normalize();
}

void ahrs_update(const float gyro_dps[3], const float accel_g[3], float dt, bool hold)
{
    float gx = hold ? 0.0f : gyro_dps[0] * DEG2RAD;
    float gy = hold ? 0.0f : gyro_dps[1] * DEG2RAD;
    float gz = hold ? 0.0f : gyro_dps[2] * DEG2RAD;

    float ax = accel_g[0], ay = accel_g[1], az = accel_g[2];
    const float an = sqrtf(ax * ax + ay * ay + az * az);
    s_accel_used = fabsf(an - 1.0f) < AHRS_ACCEL_GATE_G;
    if (s_accel_used) {
        ax /= an;
        ay /= an;
        az /= an;
        /* 由目前姿態估計的重力方向（本體座標），與量測方向的外積就是姿態誤差。 */
        const float vx = 2.0f * (q1 * q3 - q0 * q2);
        const float vy = 2.0f * (q0 * q1 + q2 * q3);
        const float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
        gx += AHRS_KP * (ay * vz - az * vy);
        gy += AHRS_KP * (az * vx - ax * vz);
        gz += AHRS_KP * (ax * vy - ay * vx);
    }

    /* q̇ = ½ q ⊗ (0, ω) */
    const float h = 0.5f * dt;
    const float a0 = q0, a1 = q1, a2 = q2, a3 = q3;
    q0 += h * (-a1 * gx - a2 * gy - a3 * gz);
    q1 += h * (a0 * gx + a2 * gz - a3 * gy);
    q2 += h * (a0 * gy - a1 * gz + a3 * gx);
    q3 += h * (a0 * gz + a1 * gy - a2 * gx);
    normalize();
}

void ahrs_euler(float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    const float sp = fmaxf(fminf(2.0f * (q0 * q2 - q3 * q1), 1.0f), -1.0f);

    *roll_deg = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD2DEG;
    *pitch_deg = asinf(sp) * RAD2DEG;
    *yaw_deg = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD2DEG;
}

bool ahrs_accel_used(void)
{
    return s_accel_used;
}
