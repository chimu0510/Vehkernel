/**
 * @file test_kinematics.c
 * @brief Unit tests for Differential Drive Kinematics and Software Heading Mapping (ADR-0001, ADR-0002).
 *
 * Designed as clear educational material for robotics kinematics:
 *   - Forward & Inverse Kinematics symmetry
 *   - Heading mapping (drive_front vs omni_front)
 *   - Encoder ticks to linear distance conversion
 */

#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "kinematics.h"

static void test_kinematics_straight_motion(void)
{
    printf("[TEST] Differential Kinematics: Straight Motion...\n");
    kinematics_config_t cfg;
    kinematics_init(&cfg, 0.350f, 0.208f, HEADING_DRIVE_FRONT);

    float v_l = 0.0f, v_r = 0.0f;
    kinematics_body_to_motors(&cfg, 1.0f, 0.0f, &v_l, &v_r);

    // Straight forward: left and right wheel velocities must be identical to vx
    assert(fabsf(v_l - 1.0f) < 1e-5f);
    assert(fabsf(v_r - 1.0f) < 1e-5f);

    float vx_out = 0.0f, wz_out = 0.0f;
    kinematics_motors_to_body(&cfg, v_l, v_r, &vx_out, &wz_out);
    assert(fabsf(vx_out - 1.0f) < 1e-5f);
    assert(fabsf(wz_out - 0.0f) < 1e-5f);
    printf("  Straight motion forward (Vx=1.0, wz=0) [PASS]\n");
}

static void test_kinematics_pure_rotation(void)
{
    printf("[TEST] Differential Kinematics: Pure In-place Rotation...\n");
    kinematics_config_t cfg;
    kinematics_init(&cfg, 0.350f, 0.208f, HEADING_DRIVE_FRONT);

    // Counter-clockwise rotation wz = +2.0 rad/s
    // v_L = Vx - (track / 2) * wz = 0 - (0.35 / 2) * 2.0 = -0.35 m/s
    // v_R = Vx + (track / 2) * wz = 0 + (0.35 / 2) * 2.0 = +0.35 m/s
    float v_l = 0.0f, v_r = 0.0f;
    kinematics_body_to_motors(&cfg, 0.0f, 2.0f, &v_l, &v_r);

    assert(fabsf(v_l - (-0.35f)) < 1e-5f);
    assert(fabsf(v_r - 0.35f) < 1e-5f);

    float vx_out = 0.0f, wz_out = 0.0f;
    kinematics_motors_to_body(&cfg, v_l, v_r, &vx_out, &wz_out);
    assert(fabsf(vx_out - 0.0f) < 1e-5f);
    assert(fabsf(wz_out - 2.0f) < 1e-5f);
    printf("  Pure in-place CCW rotation (Vx=0, wz=2.0) [PASS]\n");
}

static void test_kinematics_heading_mapping_adr0001(void)
{
    printf("[TEST] ADR-0001 Heading Mapping (drive_front vs omni_front)...\n");
    kinematics_config_t cfg_drv, cfg_omni;
    kinematics_init(&cfg_drv, 0.350f, 0.208f, HEADING_DRIVE_FRONT);
    kinematics_init(&cfg_omni, 0.350f, 0.208f, HEADING_OMNI_FRONT);

    const float target_vx = 0.8f;
    const float target_wz = 0.5f;

    // 1. DRIVE_FRONT
    float drv_l = 0.0f, drv_r = 0.0f;
    kinematics_body_to_motors(&cfg_drv, target_vx, target_wz, &drv_l, &drv_r);

    // 2. OMNI_FRONT
    // In omni_front mode, the passive omni wheels lead the vehicle.
    // Driving "forward" in body frame requires the drive wheels to spin backwards,
    // and turning perspectives are mirrored: left motor = -v_R, right motor = -v_L.
    float omni_l = 0.0f, omni_r = 0.0f;
    kinematics_body_to_motors(&cfg_omni, target_vx, target_wz, &omni_l, &omni_r);

    assert(fabsf(omni_l - (-drv_r)) < 1e-5f);
    assert(fabsf(omni_r - (-drv_l)) < 1e-5f);

    // Reconstruct body velocities from omni motor velocities
    float recon_vx = 0.0f, recon_wz = 0.0f;
    kinematics_motors_to_body(&cfg_omni, omni_l, omni_r, &recon_vx, &recon_wz);

    assert(fabsf(recon_vx - target_vx) < 1e-5f);
    assert(fabsf(recon_wz - target_wz) < 1e-5f);
    printf("  Heading mapping and inverse/forward reconstruction [PASS]\n");
}

static void test_kinematics_counts_conversions(void)
{
    printf("[TEST] Encoder ticks <-> metric distance conversion...\n");
    kinematics_config_t cfg;
    // 500 lines * 4 quadrature * 18 gear = 36000 counts per wheel revolution
    // wheel diameter = 0.208 m -> circumference = pi * 0.208 ~= 0.653451 m
    kinematics_init_full(&cfg, 0.350f, 0.208f, 36000U, HEADING_DRIVE_FRONT);

    const float expected_circ = 3.1415926535f * 0.208f;

    // Exactly 1 wheel revolution
    float dist = kinematics_counts_to_distance(&cfg, 36000);
    assert(fabsf(dist - expected_circ) < 1e-4f);

    // Reconvert distance to counts
    int32_t counts = kinematics_distance_to_counts(&cfg, dist);
    assert(counts == 36000);

    // Negative counts (reversing)
    float neg_dist = kinematics_counts_to_distance(&cfg, -18000);
    assert(fabsf(neg_dist - (-expected_circ * 0.5f)) < 1e-4f);
    int32_t neg_counts = kinematics_distance_to_counts(&cfg, neg_dist);
    assert(neg_counts == -18000);

    printf("  36000 counts = %.5f m [PASS]\n", (double)dist);
}

int main(void)
{
    printf("========================================\n");
    printf(" Running Differential Kinematics Tests\n");
    printf("========================================\n");
    test_kinematics_straight_motion();
    test_kinematics_pure_rotation();
    test_kinematics_heading_mapping_adr0001();
    test_kinematics_counts_conversions();
    printf("==> ALL KINEMATICS TESTS PASSED!\n\n");
    return 0;
}
