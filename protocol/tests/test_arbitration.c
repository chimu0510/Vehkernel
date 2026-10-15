/**
 * @file test_arbitration.c
 * @brief Unit tests for Control Source Arbitration and Failsafe Ramp State Machine (ADR-0004).
 *
 * Educational test suite verifying:
 *   - Priority ordering: SBUS (1) > Gamepad (2) > ROS 2 (3)
 *   - Timeout detection and source preemption
 *   - Failsafe deceleration ramp (prevents wheel lock-up and tipping)
 *   - Hardware Emergency Stop (E-Stop)
 */

#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "control_arbitration.h"

static void test_arbitration_priority_ordering(void)
{
    printf("[TEST] Arbitration: Priority ordering (SBUS > Gamepad > ROS 2)...\n");
    arbitration_state_t arb;
    arbitration_init(&arb, NULL);

    float out_vx = 0.0f, out_wz = 0.0f;
    uint32_t now_ms = 1000;

    // 1. Only ROS 2 active -> Mode = AUTONOMOUS, Source = ROS2
    arbitration_feed_ros2(&arb, 0.6f, 0.0f, now_ms);
    arbitration_update(&arb, 0.01f, now_ms, &out_vx, &out_wz);
    assert(arb.active_source == UGV_CONTROL_SOURCE_ROS2);
    assert(arb.current_mode == UGV_OPERATING_MODE_AUTONOMOUS);

    // 2. Gamepad activates -> Preempts ROS 2
    arbitration_feed_gamepad(&arb, 0.4f, 0.2f, true, true, now_ms);
    arbitration_update(&arb, 0.01f, now_ms, &out_vx, &out_wz);
    assert(arb.active_source == UGV_CONTROL_SOURCE_GAMEPAD);
    assert(arb.current_mode == UGV_OPERATING_MODE_MANUAL);

    // 3. SBUS activates -> Preempts Gamepad (Highest Priority)
    arbitration_feed_sbus(&arb, 0.9f, -0.3f, true, true, now_ms);
    arbitration_update(&arb, 0.01f, now_ms, &out_vx, &out_wz);
    assert(arb.active_source == UGV_CONTROL_SOURCE_SBUS);
    assert(arb.current_mode == UGV_OPERATING_MODE_MANUAL);

    printf("  Preemption sequence ROS2 -> Gamepad -> SBUS [PASS]\n");
}

static void test_arbitration_timeout_fallback(void)
{
    printf("[TEST] Arbitration: Source timeout fallback...\n");
    arbitration_state_t arb;
    arbitration_config_t cfg = {
        .max_linear_accel = 2.0f,
        .max_linear_decel = 2.0f,
        .max_angular_accel = 3.0f,
        .max_angular_decel = 3.0f,
        .sbus_timeout_ms = 100U,
        .ros2_timeout_ms = 250U,
    };
    arbitration_init(&arb, &cfg);

    float out_vx = 0.0f, out_wz = 0.0f;
    uint32_t now_ms = 1000;

    // Both SBUS and ROS 2 feed at t = 1000ms
    arbitration_feed_ros2(&arb, 0.5f, 0.0f, now_ms);
    arbitration_feed_sbus(&arb, 0.8f, 0.0f, true, true, now_ms);
    arbitration_update(&arb, 0.01f, now_ms, &out_vx, &out_wz);
    assert(arb.active_source == UGV_CONTROL_SOURCE_SBUS);

    // Advance time by 150ms (t = 1150ms). SBUS stops updating, ROS 2 keeps updating
    now_ms = 1150;
    arbitration_feed_ros2(&arb, 0.5f, 0.0f, now_ms);
    arbitration_update(&arb, 0.01f, now_ms, &out_vx, &out_wz);

    // SBUS has timed out (>100ms), fallback to ROS 2!
    assert(arb.active_source == UGV_CONTROL_SOURCE_ROS2);
    assert(arb.current_mode == UGV_OPERATING_MODE_AUTONOMOUS);
    printf("  SBUS timeout -> clean fallback to ROS2 [PASS]\n");
}

static void test_arbitration_failsafe_deceleration_ramp(void)
{
    printf("[TEST] Arbitration: Failsafe deceleration ramp (ADR-0004)...\n");
    arbitration_state_t arb;
    arbitration_config_t cfg = {
        .max_linear_accel = 10.0f,
        .max_linear_decel = 1.0f, // 1.0 m/s^2 deceleration limit
        .max_angular_accel = 10.0f,
        .max_angular_decel = 2.0f,
        .sbus_timeout_ms = 100U,
        .ros2_timeout_ms = 200U,
    };
    arbitration_init(&arb, &cfg);

    float out_vx = 0.0f, out_wz = 0.0f;
    uint32_t now_ms = 1000;

    // Bring car to 1.0 m/s in autonomous mode
    for (int i = 0; i < 20; ++i) {
        now_ms += 10;
        arbitration_feed_ros2(&arb, 1.0f, 0.0f, now_ms);
        arbitration_update(&arb, 0.01f, now_ms, &out_vx, &out_wz);
    }
    assert(fabsf(out_vx - 1.0f) < 1e-4f);

    // ROS 2 communication stops. Time advances by 250ms -> triggers timeout
    now_ms += 250;
    arbitration_update(&arb, 0.01f, now_ms, &out_vx, &out_wz);

    // Must be in FAILSAFE mode
    assert(arb.failsafe_active == true);
    assert(arb.current_mode == UGV_OPERATING_MODE_FAILSAFE);

    // Speed must NOT abruptly drop to 0!
    // In one 10ms step with max_decel = 1.0 m/s^2, speed can drop by at most 0.01 m/s:
    // 1.0 - 0.01 = 0.99 m/s
    assert(out_vx > 0.95f && out_vx < 1.0f);

    // Step 50 more times (0.5s) -> speed must ramp down smoothly to 0.5 m/s
    for (int i = 0; i < 50; ++i) {
        now_ms += 10;
        arbitration_update(&arb, 0.01f, now_ms, &out_vx, &out_wz);
    }
    assert(fabsf(out_vx - 0.49f) < 0.05f);

    // Step until vehicle is completely stopped
    for (int i = 0; i < 60; ++i) {
        now_ms += 10;
        arbitration_update(&arb, 0.01f, now_ms, &out_vx, &out_wz);
    }
    assert(fabsf(out_vx - 0.0f) < 1e-4f);
    printf("  Smooth failsafe deceleration ramp to 0 m/s [PASS]\n");
}

static void test_arbitration_estop(void)
{
    printf("[TEST] Arbitration: Emergency Stop (E-Stop)...\n");
    arbitration_state_t arb;
    arbitration_init(&arb, NULL);

    float out_vx = 0.0f, out_wz = 0.0f;
    uint32_t now_ms = 1000;

    arbitration_feed_ros2(&arb, 1.0f, 0.0f, now_ms);
    arbitration_update(&arb, 0.01f, now_ms, &out_vx, &out_wz);

    // Assert hardware E-Stop
    arbitration_set_estop(&arb, true);
    arbitration_update(&arb, 0.01f, now_ms, &out_vx, &out_wz);

    assert(arb.estop_active == true);
    assert(arb.current_mode == UGV_OPERATING_MODE_FAILSAFE);
    assert(arb.failsafe_active == true);
    printf("  Hardware E-Stop immediately engages failsafe [PASS]\n");
}

int main(void)
{
    printf("========================================\n");
    printf(" Running Control Arbitration & Failsafe Tests\n");
    printf("========================================\n");
    test_arbitration_priority_ordering();
    test_arbitration_timeout_fallback();
    test_arbitration_failsafe_deceleration_ramp();
    test_arbitration_estop();
    printf("==> ALL ARBITRATION TESTS PASSED!\n\n");
    return 0;
}
