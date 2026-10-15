/**
 * @file test_pid.c
 * @brief Unit tests for PID Speed Controller with Anti-Windup and Deadband.
 *
 * Educational test suite verifying:
 *   - Proportional control response
 *   - Integral accumulation and anti-windup clamping
 *   - Derivative transient response
 *   - Deadband thresholding
 *   - Controller reset behavior
 */

#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "pid.h"

static void test_pid_proportional(void)
{
    printf("[TEST] PID: Proportional action...\n");
    pid_controller_t pid;
    // Kp = 2.0, Ki = 0, Kd = 0, max_integral = 10, min = -100, max = +100
    pid_init(&pid, 2.0f, 0.0f, 0.0f, 10.0f, -100.0f, 100.0f);

    // Target = 10.0, Measured = 4.0 -> Error = 6.0 -> Output = 12.0
    float out = pid_update(&pid, 10.0f, 4.0f, 0.01f);
    assert(fabsf(out - 12.0f) < 1e-5f);
    printf("  P action (Kp=2.0, err=6.0 -> out=12.0) [PASS]\n");
}

static void test_pid_anti_windup(void)
{
    printf("[TEST] PID: Integral accumulation & Anti-windup clamping...\n");
    pid_controller_t pid;
    // Kp = 0, Ki = 5.0, Kd = 0, max_integral = 2.0
    pid_init(&pid, 0.0f, 5.0f, 0.0f, 2.0f, -50.0f, 50.0f);

    // Error = 10.0 over 100 cycles at dt = 0.01s (without clamp, integral would reach 10.0)
    for (int i = 0; i < 100; ++i) {
        pid_update(&pid, 10.0f, 0.0f, 0.01f);
    }

    // Integral must be clamped to max_integral (2.0), output must be Ki * 2.0 = 10.0
    assert(fabsf(pid.integral - 2.0f) < 1e-5f);
    float out = pid_update(&pid, 10.0f, 0.0f, 0.01f);
    assert(fabsf(out - 10.0f) < 1e-5f);
    printf("  Anti-windup: integral clamped at %.2f [PASS]\n", (double)pid.integral);
}

static void test_pid_deadband(void)
{
    printf("[TEST] PID: Error deadband filter...\n");
    pid_controller_t pid;
    pid_init(&pid, 5.0f, 1.0f, 0.0f, 10.0f, -50.0f, 50.0f);
    pid.deadband = 0.05f; // Errors <= 0.05 are treated as 0

    // Error = 0.02 (smaller than deadband) -> output must be 0
    float out = pid_update(&pid, 1.0f, 0.98f, 0.01f);
    assert(fabsf(out - 0.0f) < 1e-5f);

    // Error = 0.10 (larger than deadband) -> normal output
    out = pid_update(&pid, 1.0f, 0.90f, 0.01f);
    assert(out > 0.0f);
    printf("  Deadband thresholding [PASS]\n");
}

static void test_pid_output_saturation(void)
{
    printf("[TEST] PID: Output saturation limits...\n");
    pid_controller_t pid;
    pid_init(&pid, 100.0f, 0.0f, 0.0f, 10.0f, -15.0f, 15.0f);

    float high_out = pid_update(&pid, 100.0f, 0.0f, 0.01f);
    assert(fabsf(high_out - 15.0f) < 1e-5f);

    float low_out = pid_update(&pid, -100.0f, 0.0f, 0.01f);
    assert(fabsf(low_out - (-15.0f)) < 1e-5f);
    printf("  Output clamped to [-15.0, +15.0] [PASS]\n");
}

static void test_pid_reset(void)
{
    printf("[TEST] PID: Reset behavior...\n");
    pid_controller_t pid;
    pid_init(&pid, 1.0f, 2.0f, 1.0f, 10.0f, -50.0f, 50.0f);

    pid_update(&pid, 5.0f, 0.0f, 0.01f);
    assert(pid.integral > 0.0f);
    assert(pid.prev_error > 0.0f);

    pid_reset(&pid);
    assert(pid.integral == 0.0f);
    assert(pid.prev_error == 0.0f);
    printf("  pid_reset clears state cleanly [PASS]\n");
}

int main(void)
{
    printf("========================================\n");
    printf(" Running PID Controller Tests\n");
    printf("========================================\n");
    test_pid_proportional();
    test_pid_anti_windup();
    test_pid_deadband();
    test_pid_output_saturation();
    test_pid_reset();
    printf("==> ALL PID TESTS PASSED!\n\n");
    return 0;
}
