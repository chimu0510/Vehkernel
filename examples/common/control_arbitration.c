#include "control_arbitration.h"

#include <math.h>

static float ramp_limit(float current, float target, float max_accel, float max_decel, float dt)
{
    if (dt <= 0.0f) {
        return current;
    }

    const float delta = target - current;
    float max_step;

    /* 判斷是加速還是減速 */
    const bool is_accelerating = (current >= 0.0f && delta > 0.0f) || (current <= 0.0f && delta < 0.0f);
    if (is_accelerating) {
        max_step = max_accel * dt;
    } else {
        max_step = max_decel * dt;
    }

    if (delta > max_step) {
        return current + max_step;
    } else if (delta < -max_step) {
        return current - max_step;
    } else {
        return target;
    }
}

void arbitration_init(arbitration_state_t *arb, const arbitration_config_t *cfg)
{
    if (cfg != NULL) {
        arb->config = *cfg;
    } else {
        arb->config.max_linear_accel = 2.0f;    /* 2.0 m/s^2 */
        arb->config.max_linear_decel = 1.5f;    /* 1.5 m/s^2 斜坡減速 */
        arb->config.max_angular_accel = 3.0f;   /* 3.0 rad/s^2 */
        arb->config.max_angular_decel = 2.5f;   /* 2.5 rad/s^2 */
        arb->config.sbus_timeout_ms = 100U;
        arb->config.ros2_timeout_ms = 250U;
    }

    arb->sbus = (control_input_t){0};
    arb->gamepad = (control_input_t){0};
    arb->ros2 = (control_input_t){0};

    arb->active_source = UGV_CONTROL_SOURCE_NONE;
    arb->current_mode = UGV_OPERATING_MODE_MANUAL;
    arb->cmd_vx = 0.0f;
    arb->cmd_wz = 0.0f;

    arb->failsafe_active = false;
    arb->estop_active = false;
    arb->low_battery_active = false;
}

void arbitration_feed_sbus(arbitration_state_t *arb, float vx, float wz, bool active, bool healthy, uint32_t now_ms)
{
    arb->sbus.vx = vx;
    arb->sbus.wz = wz;
    arb->sbus.active = active;
    arb->sbus.healthy = healthy;
    arb->sbus.last_rx_ms = now_ms;
}

void arbitration_feed_gamepad(arbitration_state_t *arb, float vx, float wz, bool active, bool healthy, uint32_t now_ms)
{
    arb->gamepad.vx = vx;
    arb->gamepad.wz = wz;
    arb->gamepad.active = active;
    arb->gamepad.healthy = healthy;
    arb->gamepad.last_rx_ms = now_ms;
}

void arbitration_feed_ros2(arbitration_state_t *arb, float vx, float wz, uint32_t now_ms)
{
    arb->ros2.vx = vx;
    arb->ros2.wz = wz;
    arb->ros2.active = true;
    arb->ros2.healthy = true;
    arb->ros2.last_rx_ms = now_ms;
}

void arbitration_set_estop(arbitration_state_t *arb, bool estop)
{
    arb->estop_active = estop;
}

void arbitration_set_low_battery(arbitration_state_t *arb, bool low_battery)
{
    arb->low_battery_active = low_battery;
}

void arbitration_update(arbitration_state_t *arb, float dt, uint32_t now_ms, float *out_vx, float *out_wz)
{
    /* 檢查逾時狀態 */
    if (now_ms - arb->sbus.last_rx_ms > arb->config.sbus_timeout_ms) {
        arb->sbus.healthy = false;
        arb->sbus.active = false;
    }
    if (now_ms - arb->ros2.last_rx_ms > arb->config.ros2_timeout_ms) {
        arb->ros2.healthy = false;
    }

    float target_vx = 0.0f;
    float target_wz = 0.0f;
    bool require_failsafe = false;

    /* 檢查嚴重異常：急停開關斷開或低電量持續 */
    if (arb->estop_active || arb->low_battery_active) {
        require_failsafe = true;
    }

    if (!require_failsafe) {
        /*
         * ADR-0004 固定優先序仲裁:
         * 1. SBUS 活躍 -> 接管
         * 2. USB 手柄活躍 -> 接管
         * 3. ROS2 健康 -> 自主模式
         * 4. 無輸入 -> 停車 (若是自主模式中途失聯則進入失效保護)
         */
        if (arb->sbus.healthy && arb->sbus.active) {
            arb->active_source = UGV_CONTROL_SOURCE_SBUS;
            arb->current_mode = UGV_OPERATING_MODE_MANUAL;
            target_vx = arb->sbus.vx;
            target_wz = arb->sbus.wz;
            arb->failsafe_active = false;
        } else if (arb->gamepad.healthy && arb->gamepad.active) {
            arb->active_source = UGV_CONTROL_SOURCE_GAMEPAD;
            arb->current_mode = UGV_OPERATING_MODE_MANUAL;
            target_vx = arb->gamepad.vx;
            target_wz = arb->gamepad.wz;
            arb->failsafe_active = false;
        } else if (arb->ros2.healthy) {
            arb->active_source = UGV_CONTROL_SOURCE_ROS2;
            arb->current_mode = UGV_OPERATING_MODE_AUTONOMOUS;
            target_vx = arb->ros2.vx;
            target_wz = arb->ros2.wz;
            arb->failsafe_active = false;
        } else {
            /* 如果先前在自主模式，突然中斷失去心跳，標記為失效保護 */
            if (arb->current_mode == UGV_OPERATING_MODE_AUTONOMOUS) {
                require_failsafe = true;
            }
            arb->active_source = UGV_CONTROL_SOURCE_NONE;
            target_vx = 0.0f;
            target_wz = 0.0f;
        }
    }

    if (require_failsafe) {
        arb->failsafe_active = true;
        arb->current_mode = UGV_OPERATING_MODE_FAILSAFE;
        arb->active_source = UGV_CONTROL_SOURCE_NONE;
        target_vx = 0.0f;
        target_wz = 0.0f;
    }

    /*
     * ADR-0004: 斜坡減速停車 (Ramped Failsafe)
     * 使用設定的最大減速度將車速平滑降至 0
     */
    arb->cmd_vx = ramp_limit(arb->cmd_vx, target_vx,
                             arb->config.max_linear_accel,
                             arb->config.max_linear_decel,
                             dt);
    arb->cmd_wz = ramp_limit(arb->cmd_wz, target_wz,
                             arb->config.max_angular_accel,
                             arb->config.max_angular_decel,
                             dt);

    if (out_vx != NULL) {
        *out_vx = arb->cmd_vx;
    }
    if (out_wz != NULL) {
        *out_wz = arb->cmd_wz;
    }
}
