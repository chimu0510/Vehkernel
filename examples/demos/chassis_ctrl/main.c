/*
 * chassis_ctrl: C50C 生產級底盤整合控制韌體
 *
 * 整合功能：
 *   - 差速運動學正逆解算與軟體車頭方向映射 (ADR-0001)
 *   - 100 Hz 左右輪速閉環 PID 控制與 ICM-20948 wz 偏航角速度閉環 (ADR-0002)
 *   - 高速 DMA 二進位串口通訊協議 (ADR-0003: 50 Hz 遙測、微秒時間戳、握手版本與車頭校驗)
 *   - 固定優先序控制來源仲裁 SBUS > USB 手柄 > ROS 2 與斜坡減速失效保護 (ADR-0004)
 *   - 板載 OLED 即時狀態儀表、電池電壓四級監控與使能開關急停保護
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "board.h"
#include "board_id.h"
#include "settings.h"
#include "motor.h"
#include "encoder.h"
#include "imu.h"
#include "battery_monitor.h"
#include "oled.h"
#include "ctrl_tick.h"
#include "pid.h"
#include "kinematics.h"
#include "yaw_rate_loop.h"
#include "control_arbitration.h"
#include "sbus.h"
#include "ugv_link.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* 全域狀態與控制器 */
static settings_t s_settings;
static board_id_t s_board_id;
static kinematics_config_t s_kinematics;
static pid_controller_t s_pid_l;
static pid_controller_t s_pid_r;
static yaw_rate_loop_t s_yaw_loop;
static arbitration_state_t s_arb;

/* 100 Hz 中斷量測量 */
static volatile float s_meas_vx = 0.0f;
static volatile float s_meas_wz = 0.0f;
static volatile float s_wheel_vel_l = 0.0f;
static volatile float s_wheel_vel_r = 0.0f;
static volatile int32_t s_wheel_pos_l = 0;
static volatile int32_t s_wheel_pos_r = 0;
static volatile float s_accel_g[3] = {0};
static volatile float s_gyro_dps[3] = {0};

/* 編碼器差分計算 */
static int32_t s_last_enc[ENC_COUNT] = {0};

/* 通訊序號與統計 */
static uint32_t s_telem_seq = 0;
static uint32_t s_last_telem_ms = 0;
static uint32_t s_last_oled_ms = 0;

static void on_ctrl_tick(void)
{
    const float dt = 0.01f; /* 100 Hz 控制週期 = 10 ms */
    const uint32_t now_ms = HAL_GetTick();

    /* 1. 讀取編碼器累計計數並計算週期增量 */
    int32_t cur_enc[ENC_COUNT];
    encoder_poll(cur_enc);

    const int32_t d_counts_l = (cur_enc[ENC_A] - s_last_enc[ENC_A]) * WHEEL_LEFT_ENC_SIGN;
    const int32_t d_counts_r = (cur_enc[ENC_B] - s_last_enc[ENC_B]) * WHEEL_RIGHT_ENC_SIGN;
    s_last_enc[ENC_A] = cur_enc[ENC_A];
    s_last_enc[ENC_B] = cur_enc[ENC_B];

    s_wheel_pos_l = cur_enc[ENC_A] * WHEEL_LEFT_ENC_SIGN;
    s_wheel_pos_r = cur_enc[ENC_B] * WHEEL_RIGHT_ENC_SIGN;

    /* 2. 計算驅動端左右輪線速度 (m/s) 與車體量測速度 (Vx, wz) */
    const float vel_l = kinematics_counts_to_distance(&s_kinematics, d_counts_l) / dt;
    const float vel_r = kinematics_counts_to_distance(&s_kinematics, d_counts_r) / dt;
    s_wheel_vel_l = vel_l;
    s_wheel_vel_r = vel_r;

    float vx_meas = 0.0f, wz_meas = 0.0f;
    kinematics_motors_to_body(&s_kinematics, vel_l, vel_r, &vx_meas, &wz_meas);
    s_meas_vx = vx_meas;
    s_meas_wz = wz_meas;

    /* 3. 讀取 IMU 加速度與角速度 */
    imu_sample_t imu_data;
    if (imu_read(&imu_data)) {
        for (int i = 0; i < 3; i++) {
            s_accel_g[i] = imu_data.accel_g[i];
            s_gyro_dps[i] = imu_data.gyro_dps[i];
        }
    }
    const float gyro_wz_rad = (s_gyro_dps[2] * (float)M_PI / 180.0f) * (float)IMU_YAW_SIGN;

    /* 4. 更新安全開關與電池狀態 */
    const bool estop = !enable_switch_high();
    arbitration_set_estop(&s_arb, estop);

    const batt_status_t *bat = battery_monitor_status();
    arbitration_set_low_battery(&s_arb, bat->level >= BATT_LOW);

    /* 5. 讀取 SBUS 遙控器輸入 */
    sbus_state_t sbus_state;
    if (sbus_read(&sbus_state)) {
        float sbus_vx = 0.0f, sbus_wz = 0.0f;
        bool sbus_active = false;
        sbus_get_velocity(&sbus_state, 1.2f, 2.5f, &sbus_vx, &sbus_wz, &sbus_active);
        arbitration_feed_sbus(&s_arb, sbus_vx, sbus_wz, sbus_active,
                              !sbus_state.failsafe && !sbus_state.frame_lost, now_ms);
    }

    /* 6. 控制來源仲裁與斜坡減速 (ADR-0004) */
    float cmd_vx = 0.0f, cmd_wz = 0.0f;
    arbitration_update(&s_arb, dt, now_ms, &cmd_vx, &cmd_wz);

    /* 7. 偏航角速度 wz 閉環補償 (ADR-0002: 車體運動時啟用，靜止時重設以避免零偏漂移擾動) */
    float cmd_wz_corrected = cmd_wz;
    if (fabsf(cmd_vx) > 0.01f || fabsf(cmd_wz) > 0.01f) {
        cmd_wz_corrected = yaw_rate_loop_update(&s_yaw_loop, cmd_wz, gyro_wz_rad, dt);
    } else {
        yaw_rate_loop_reset(&s_yaw_loop);
    }

    /* 8. 逆運動學：由車體 (Vx, wz) 映射解算左右馬達目標速度 (ADR-0001) */
    float target_motor_l = 0.0f, target_motor_r = 0.0f;
    kinematics_body_to_motors(&s_kinematics, cmd_vx, cmd_wz_corrected, &target_motor_l, &target_motor_r);

    /* 9. 輪速 PID 閉環控制 */
    const float duty_l = pid_update(&s_pid_l, target_motor_l, vel_l, dt);
    const float duty_r = pid_update(&s_pid_r, target_motor_r, vel_r, dt);

    /*
     * 10. 馬達安全輸出保護 (ADR-0004):
     * - 急停開關切斷 (estop) -> 強制停機
     * - 無活躍控制來源 (active_source == NONE):
     *     若處於失效保護減速斜坡中 (failsafe_active 且車速 >= 0.01) -> 允許減速斜坡
     *     其餘情況 (開機待機、減速完成、失聯無指令) -> 強制鎖定停機 (motor_stop_all) 並重設 PID
     */
    const bool is_ramping_failsafe = s_arb.failsafe_active &&
                                     (fabsf(s_arb.cmd_vx) >= 0.01f || fabsf(s_arb.cmd_wz) >= 0.01f);
    const bool allow_motor_drive = !estop &&
                                   ((s_arb.active_source != UGV_CONTROL_SOURCE_NONE) || is_ramping_failsafe);

    if (!allow_motor_drive) {
        motor_stop_all();
        pid_reset(&s_pid_l);
        pid_reset(&s_pid_r);
        yaw_rate_loop_reset(&s_yaw_loop);

        if (estop) {
            led_set(true, false, false); /* 紅燈: 急停開關關閉 */
        } else if (s_arb.failsafe_active) {
            led_set(true, true, false);  /* 黃燈: 失效保護煞停完成 */
        } else {
            led_set(false, false, false); /* 熄滅: 待機無控制來源，馬達完全安全脫機鎖定 */
        }
    } else {
        motor_set(MOTOR_A, duty_l * (float)WHEEL_LEFT_MOTOR_SIGN);
        motor_set(MOTOR_B, duty_r * (float)WHEEL_RIGHT_MOTOR_SIGN);

        /* LED 狀態指示: 藍色=自主模式(ROS2), 綠色=手動遙控(SBUS), 黃色=減速斜坡中 */
        if (s_arb.active_source == UGV_CONTROL_SOURCE_ROS2) {
            led_set(false, false, true);
        } else if (s_arb.active_source == UGV_CONTROL_SOURCE_SBUS) {
            led_set(false, true, false);
        } else if (s_arb.failsafe_active) {
            led_set(true, true, false);
        } else {
            led_set(false, false, false);
        }
    }
}

static void handle_rx_messages(void)
{
    ugv_msg_payload_t msg;
    const size_t len = ugv_link_poll(&msg);
    if (len == 0) {
        return;
    }

    const uint32_t now_ms = HAL_GetTick();

    switch (msg.header.msg_id) {
    case UGV_MSG_HANDSHAKE_REQ: {
        ugv_msg_handshake_resp_t resp = {0};
        resp.header.msg_id = UGV_MSG_HANDSHAKE_RESP;
        resp.header.seq = msg.header.seq;
        resp.header.timestamp_us = micros();
        resp.mcu_proto_version = UGV_PROTOCOL_VERSION;
        resp.mcu_heading = (uint8_t)s_settings.heading;
        resp.heading_matched = (msg.handshake_req.expected_heading == resp.mcu_heading) ? 1U : 0U;
        resp.hw_version = (uint8_t)s_board_id.version;
        memcpy(resp.mcu_uid, (const void *)0x1FFF7A10, 12);
        ugv_link_send(&resp, sizeof(resp));
        break;
    }

    case UGV_MSG_HEARTBEAT: {
        /* 上位機心跳，維持自主模式連線 */
        arbitration_feed_ros2(&s_arb, s_arb.ros2.vx, s_arb.ros2.wz, now_ms);

        const batt_status_t *bat = battery_monitor_status();
        ugv_msg_heartbeat_ack_t ack = {0};
        ack.header.msg_id = UGV_MSG_HEARTBEAT_ACK;
        ack.header.seq = msg.header.seq;
        ack.header.timestamp_us = micros();
        ack.echo_host_seq = msg.header.seq;
        ack.active_control_source = s_arb.active_source;
        ack.current_mode = s_arb.current_mode;
        ack.status_flags = (uint16_t)(
            (!enable_switch_high() ? UGV_SYSTEM_STATUS_ESTOP_ACTIVE : 0) |
            (s_arb.failsafe_active ? UGV_SYSTEM_STATUS_FAILSAFE_ACTIVE : 0) |
            (bat->level >= BATT_LOW ? UGV_SYSTEM_STATUS_LOW_BATTERY : 0) |
            UGV_SYSTEM_STATUS_IMU_READY
        );
        ugv_link_send(&ack, sizeof(ack));
        break;
    }

    case UGV_MSG_SET_VELOCITY: {
        /* 下發車體速度 (ADR-0002) */
        arbitration_feed_ros2(&s_arb, msg.set_velocity.target_vx, msg.set_velocity.target_wz, now_ms);
        break;
    }

    case UGV_MSG_SET_PID_CONFIG: {
        /* 動態設定 PID 參數 */
        s_pid_l.kp = msg.set_pid_config.wheel_kp;
        s_pid_l.ki = msg.set_pid_config.wheel_ki;
        s_pid_l.kd = msg.set_pid_config.wheel_kd;
        s_pid_r.kp = msg.set_pid_config.wheel_kp;
        s_pid_r.ki = msg.set_pid_config.wheel_ki;
        s_pid_r.kd = msg.set_pid_config.wheel_kd;
        s_yaw_loop.pid.kp = msg.set_pid_config.yaw_kp;
        s_yaw_loop.pid.ki = msg.set_pid_config.yaw_ki;
        s_yaw_loop.pid.kd = msg.set_pid_config.yaw_kd;
        s_arb.config.max_linear_accel = msg.set_pid_config.max_accel;
        s_arb.config.max_linear_decel = msg.set_pid_config.max_decel;

        ugv_msg_pid_config_resp_t resp = {0};
        resp.header.msg_id = UGV_MSG_PID_CONFIG_RESP;
        resp.header.seq = msg.header.seq;
        resp.header.timestamp_us = micros();
        resp.wheel_kp = s_pid_l.kp;
        resp.wheel_ki = s_pid_l.ki;
        resp.wheel_kd = s_pid_l.kd;
        resp.yaw_kp = s_yaw_loop.pid.kp;
        resp.yaw_ki = s_yaw_loop.pid.ki;
        resp.yaw_kd = s_yaw_loop.pid.kd;
        resp.max_accel = s_arb.config.max_linear_accel;
        resp.max_decel = s_arb.config.max_linear_decel;
        ugv_link_send(&resp, sizeof(resp));
        break;
    }

    case UGV_MSG_SET_HEADING_CONFIG: {
        /* 車頭方向設定 (ADR-0001: 寫入 Flash 重啟生效) */
        ugv_msg_heading_config_resp_t resp = {0};
        resp.header.msg_id = UGV_MSG_HEADING_CONFIG_RESP;
        resp.header.seq = msg.header.seq;
        resp.header.timestamp_us = micros();
        resp.current_heading = (uint8_t)s_settings.heading;

        if (msg.set_heading_config.save_to_flash) {
            settings_t target_s;
            target_s.heading = (heading_t)msg.set_heading_config.target_heading;
            if (settings_save(&target_s)) {
                resp.flash_heading = (uint8_t)target_s.heading;
                resp.write_result = 1U; /* 寫入成功需重啟 */
            } else {
                resp.flash_heading = (uint8_t)s_settings.heading;
                resp.write_result = 2U; /* 寫入失敗 */
            }
        } else {
            resp.flash_heading = (uint8_t)s_settings.heading;
            resp.write_result = 0U;
        }
        ugv_link_send(&resp, sizeof(resp));
        break;
    }

    default:
        break;
    }
}

static void send_telemetry_50hz(void)
{
    const uint32_t now_ms = HAL_GetTick();
    if (now_ms - s_last_telem_ms < 20U) { /* 20 ms = 50 Hz */
        return;
    }
    s_last_telem_ms = now_ms;

    ugv_msg_telemetry_t telem = {0};
    telem.header.msg_id = UGV_MSG_TELEMETRY;
    telem.header.seq = s_telem_seq++;
    telem.header.timestamp_us = micros();

    telem.measured_vx = s_meas_vx;
    telem.measured_wz = s_meas_wz;
    telem.wheel_vel_l = s_wheel_vel_l;
    telem.wheel_vel_r = s_wheel_vel_r;
    telem.wheel_pos_l = s_wheel_pos_l;
    telem.wheel_pos_r = s_wheel_pos_r;

    telem.imu_accel_x = s_accel_g[0] * 9.80665f;
    telem.imu_accel_y = s_accel_g[1] * 9.80665f;
    telem.imu_accel_z = s_accel_g[2] * 9.80665f;
    telem.imu_gyro_x = s_gyro_dps[0] * (float)M_PI / 180.0f;
    telem.imu_gyro_y = s_gyro_dps[1] * (float)M_PI / 180.0f;
    telem.imu_gyro_z = s_gyro_dps[2] * (float)M_PI / 180.0f;

    const batt_status_t *bat = battery_monitor_status();
    telem.battery_voltage = bat->vbat_filtered;
    telem.battery_level = (uint8_t)bat->level;
    telem.control_source = (uint8_t)s_arb.active_source;

    telem.status_flags = (uint16_t)(
        (!enable_switch_high() ? UGV_SYSTEM_STATUS_ESTOP_ACTIVE : 0) |
        (s_arb.failsafe_active ? UGV_SYSTEM_STATUS_FAILSAFE_ACTIVE : 0) |
        (bat->level >= BATT_LOW ? UGV_SYSTEM_STATUS_LOW_BATTERY : 0) |
        UGV_SYSTEM_STATUS_IMU_READY
    );

    ugv_link_send(&telem, sizeof(telem));
}

static void update_oled_display_10hz(void)
{
    const uint32_t now_ms = HAL_GetTick();
    if (now_ms - s_last_oled_ms < 100U) { /* 100 ms = 10 Hz */
        return;
    }
    s_last_oled_ms = now_ms;

    oled_clear();

    /* 第 0 行: 車型與車頭方向 + 模式 */
    char line1[24];
    const char *hd_str = (s_settings.heading == HEADING_DRIVE_FRONT) ? "DRV" : "OMN";
    const char *mode_str = (s_arb.current_mode == UGV_OPERATING_MODE_AUTONOMOUS) ? "AUTO" :
                           (s_arb.current_mode == UGV_OPERATING_MODE_FAILSAFE) ? "FAIL" : "MANU";
    snprintf(line1, sizeof(line1), "UGV [%s] [%s]", hd_str, mode_str);
    oled_text(0, 0, line1);

    /* 第 2 行: 控制來源 */
    char line2[24];
    const char *src_str = (s_arb.active_source == UGV_CONTROL_SOURCE_ROS2) ? "ROS2" :
                          (s_arb.active_source == UGV_CONTROL_SOURCE_SBUS) ? "SBUS" :
                          (s_arb.active_source == UGV_CONTROL_SOURCE_GAMEPAD) ? "PAD" : "NONE";
    snprintf(line2, sizeof(line2), "Src:%s %s", src_str, enable_switch_high() ? "ARM" : "ESTOP");
    oled_text(0, 2, line2);

    /* 第 4 行: 線速度與角速度 */
    char line3[24];
    snprintf(line3, sizeof(line3), "V:%+.2f W:%+.2f", (double)s_meas_vx, (double)s_meas_wz);
    oled_text(0, 4, line3);

    /* 第 6 行: 電池電壓與健康等級 */
    char line4[24];
    const batt_status_t *bat = battery_monitor_status();
    snprintf(line4, sizeof(line4), "Bat:%.1fV [%s]", (double)bat->vbat_filtered,
             batt_level_name(bat->level));
    oled_text(0, 6, line4);

    oled_flush();
}

int main(void)
{
    board_init();

    /* 辨識硬體版本 */
    s_board_id = board_id_detect();

    /* 載入 Flash 設定 (ADR-0001) */
    settings_load(&s_settings);

    oled_init();
    oled_clear();
    oled_text(0, 2, "C50C Chassis Init...");
    oled_flush();

    motor_init();
    encoder_init();
    imu_init();
    battery_monitor_init();
    sbus_init();

    /* 初始化通訊協議鏈路 (ADR-0003) */
    ugv_link_init(UGV_DEFAULT_BAUD);

    /* 初始化差速運動學 (ADR-0001, ADR-0002) */
    kinematics_init(&s_kinematics, DEFAULT_TRACK_WIDTH_M, DEFAULT_WHEEL_DIAMETER_M, s_settings.heading);

    /* 初始化輪速 PID 控制器 (Kp=0.45, Ki=1.2, Kd=0.01) */
    pid_init(&s_pid_l, 0.45f, 1.20f, 0.01f, 0.8f, -1.0f, 1.0f);
    pid_init(&s_pid_r, 0.45f, 1.20f, 0.01f, 0.8f, -1.0f, 1.0f);

    /* 初始化偏航角速度閉環控制器 (Kp=0.25, Ki=0.05, Kd=0.0) */
    yaw_rate_loop_init(&s_yaw_loop, 0.25f, 0.05f, 0.00f, 1.0f);

    /* 初始化控制來源仲裁與失效保護狀態機 (ADR-0004) */
    arbitration_init(&s_arb, NULL);

    /* 啟動 100 Hz 定頻控制中斷 */
    ctrl_tick_start(100U, on_ctrl_tick);

    /* 主迴路：處理通訊封包、50 Hz 遙測、電池監控與 10 Hz OLED 顯示 */
    while (1) {
        battery_monitor_update();
        handle_rx_messages();
        send_telemetry_50hz();
        update_oled_display_10hz();
    }
}
