#include "pid.h"

#include <math.h>

void pid_init(pid_controller_t *pid, float kp, float ki, float kd,
              float max_integral, float min_output, float max_output)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->max_integral = fabsf(max_integral);
    pid->min_output = min_output;
    pid->max_output = max_output;
    pid->deadband = 0.0f;
}

void pid_reset(pid_controller_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

float pid_update(pid_controller_t *pid, float setpoint, float measurement, float dt)
{
    if (dt <= 0.0f) {
        return 0.0f;
    }

    float error = setpoint - measurement;
    if (fabsf(error) < pid->deadband) {
        error = 0.0f;
    }

    /* 積分更新（帶抗飽和限制） */
    pid->integral += error * dt;
    if (pid->integral > pid->max_integral) {
        pid->integral = pid->max_integral;
    } else if (pid->integral < -pid->max_integral) {
        pid->integral = -pid->max_integral;
    }

    /* 微分計算 */
    const float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    float output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);

    /* 輸出限制 */
    if (output > pid->max_output) {
        output = pid->max_output;
    } else if (output < pid->min_output) {
        output = pid->min_output;
    }

    return output;
}
