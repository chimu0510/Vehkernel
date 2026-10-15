/*
 * 通用 PID 控制器 (速度閉環與角速度閉環)
 */
#ifndef PID_H
#define PID_H

#include <stdbool.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float max_integral;
    float min_output;
    float max_output;
    float deadband;      /* 誤差死區，小於此值視為 0 */
} pid_controller_t;

void pid_init(pid_controller_t *pid, float kp, float ki, float kd,
              float max_integral, float min_output, float max_output);

void pid_reset(pid_controller_t *pid);

float pid_update(pid_controller_t *pid, float setpoint, float measurement, float dt);

#endif /* PID_H */
