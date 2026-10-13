#include "yaw_rate_loop.h"

#include <math.h>

void yaw_rate_loop_init(yaw_rate_loop_t *loop, float kp, float ki, float kd, float max_correction)
{
    pid_init(&loop->pid, kp, ki, kd, max_correction, -max_correction, max_correction);
    loop->max_correction_wz = fabsf(max_correction);
    loop->enabled = true;
}

void yaw_rate_loop_reset(yaw_rate_loop_t *loop)
{
    pid_reset(&loop->pid);
}

void yaw_rate_loop_enable(yaw_rate_loop_t *loop, bool enable)
{
    loop->enabled = enable;
    if (!enable) {
        pid_reset(&loop->pid);
    }
}

float yaw_rate_loop_update(yaw_rate_loop_t *loop, float target_wz, float measured_gyro_wz, float dt)
{
    if (!loop->enabled) {
        return target_wz;
    }

    /*
     * 目標角速度與陀螺儀讀值之間的誤差：
     * error = target_wz - measured_gyro_wz
     * 補償量 correction 由 PID 產生
     */
    const float correction = pid_update(&loop->pid, target_wz, measured_gyro_wz, dt);

    return target_wz + correction;
}
