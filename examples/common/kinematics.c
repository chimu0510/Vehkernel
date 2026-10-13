#include "kinematics.h"

#include <math.h>
#include "board.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

void kinematics_init(kinematics_config_t *cfg, float track_width, float wheel_diameter, heading_t heading)
{
    cfg->track_width = track_width > 0.05f ? track_width : DEFAULT_TRACK_WIDTH_M;
    cfg->wheel_diameter = wheel_diameter > 0.05f ? wheel_diameter : DEFAULT_WHEEL_DIAMETER_M;
    cfg->heading = heading;
}

void kinematics_body_to_motors(const kinematics_config_t *cfg, float vx, float wz,
                               float *out_motor_l_vel, float *out_motor_r_vel)
{
    /*
     * 差速運動學：
     * v_L = Vx - (L / 2) * wz
     * v_R = Vx + (L / 2) * wz
     */
    const float half_track = cfg->track_width * 0.5f;
    const float v_l = vx - (half_track * wz);
    const float v_r = vx + (half_track * wz);

    /*
     * ADR-0001: 映射層切換
     * 驅動端在前 (DRIVE_END_FRONT):
     *   左馬達 = v_L, 右馬達 = v_R
     * 全向輪端在前 (OMNI_END_FRONT):
     *   以全向輪端為車頭向前行駛，驅動端輪子需反轉後退，且左右視角相反：
     *   左馬達 = -v_R, 右馬達 = -v_L
     */
    if (cfg->heading == HEADING_OMNI_END_FRONT) {
        *out_motor_l_vel = -v_r;
        *out_motor_r_vel = -v_l;
    } else {
        *out_motor_l_vel = v_l;
        *out_motor_r_vel = v_r;
    }
}

void kinematics_motors_to_body(const kinematics_config_t *cfg, float motor_l_vel, float motor_r_vel,
                               float *out_vx, float *out_wz)
{
    float v_l, v_r;

    if (cfg->heading == HEADING_OMNI_END_FRONT) {
        v_l = -motor_r_vel;
        v_r = -motor_l_vel;
    } else {
        v_l = motor_l_vel;
        v_r = motor_r_vel;
    }

    *out_vx = (v_r + v_l) * 0.5f;
    *out_wz = (v_r - v_l) / cfg->track_width;
}

float kinematics_counts_to_distance(const kinematics_config_t *cfg, int32_t counts)
{
    const float circumference = (float)M_PI * cfg->wheel_diameter;
    return ((float)counts / (float)COUNTS_PER_WHEEL_REV) * circumference;
}

int32_t kinematics_distance_to_counts(const kinematics_config_t *cfg, float distance_m)
{
    const float circumference = (float)M_PI * cfg->wheel_diameter;
    if (circumference <= 0.0f) {
        return 0;
    }
    return (int32_t)((distance_m / circumference) * (float)COUNTS_PER_WHEEL_REV);
}
