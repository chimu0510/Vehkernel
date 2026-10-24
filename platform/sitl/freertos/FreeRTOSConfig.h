// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * FreeRTOS configuration for SITL (POSIX port, ADR-0018).
 *
 * Kept as close to the STM32F4 configuration as the port allows, so
 * scheduling behaves as on hardware. Each task runs on a pthread, so stacks
 * must hold a host thread's frames.
 */
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* Scheduler. */
#define configTICK_RATE_HZ            1000
#define configUSE_PREEMPTION          1
#define configUSE_TIME_SLICING        1
#define configMAX_PRIORITIES          8
#define configMINIMAL_STACK_SIZE      8192
#define configMAX_TASK_NAME_LEN       12
#define configTICK_TYPE_WIDTH_IN_BITS TICK_TYPE_WIDTH_64_BITS
#define configIDLE_SHOULD_YIELD       1

/* Memory: static allocation only, as on hardware. */
#define configSUPPORT_STATIC_ALLOCATION  1
#define configSUPPORT_DYNAMIC_ALLOCATION 0

/* Where core/ places task stacks: anywhere on the host. */
#define vkconfigTASK_STACK_ATTRIBUTE
/* Stack depth, in words, of each work queue task. */
#define vkconfigWORK_QUEUE_STACK_SIZE configMINIMAL_STACK_SIZE

/* Features. */
#define configUSE_MUTEXES             1
#define configUSE_RECURSIVE_MUTEXES   0
#define configUSE_COUNTING_SEMAPHORES 1
#define configUSE_TASK_NOTIFICATIONS  1
#define configUSE_TIMERS              0
#define configUSE_CO_ROUTINES         0
#define configQUEUE_REGISTRY_SIZE     0

/* Hooks. */
#define configUSE_IDLE_HOOK            0
#define configUSE_TICK_HOOK            0
#define configUSE_MALLOC_FAILED_HOOK   0
#define configCHECK_FOR_STACK_OVERFLOW 0

/* API functions compiled in. */
#define INCLUDE_vTaskDelay                  1
#define INCLUDE_xTaskDelayUntil             1
#define INCLUDE_vTaskSuspend                1
#define INCLUDE_vTaskDelete                 1
#define INCLUDE_xTaskGetCurrentTaskHandle   1
#define INCLUDE_xTaskGetSchedulerState      1
#define INCLUDE_uxTaskGetStackHighWaterMark 1

/*
 * A failed assertion goes to the fault stop (platform.cpp, ADR-0005), which
 * turns the simulated outputs off and ends the run, as on hardware.
 */
#ifdef __cplusplus
extern "C" {
#endif
void vk_assert_failed(void) __attribute__((noreturn));
#ifdef __cplusplus
}
#endif
#define configASSERT(x)     \
    if ((x) == 0) {         \
        vk_assert_failed(); \
    }

#endif /* FREERTOS_CONFIG_H */
