// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * FreeRTOS configuration for STM32F4 (Cortex-M4F port, ADR-0004).
 *
 * Everything is allocated statically (ADR-0007, ADR-0008): dynamic allocation
 * is compiled out, so no FreeRTOS heap exists.
 */
#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* Scheduler. The system clock comes from the board definition (VK_SYSCLK_HZ). */
#define configCPU_CLOCK_HZ                      VK_SYSCLK_HZ
#define configTICK_RATE_HZ                      1000
#define configUSE_PREEMPTION                    1
#define configUSE_TIME_SLICING                  1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configMAX_PRIORITIES                    8
#define configMINIMAL_STACK_SIZE                128
#define configMAX_TASK_NAME_LEN                 12
#define configTICK_TYPE_WIDTH_IN_BITS           TICK_TYPE_WIDTH_32_BITS
#define configIDLE_SHOULD_YIELD                 1

/* Memory: static allocation only. */
#define configSUPPORT_STATIC_ALLOCATION  1
#define configSUPPORT_DYNAMIC_ALLOCATION 0

/* Where core/ places task stacks: CCM, leaving normal SRAM for DMA (ADR-0008). */
#define vkconfigTASK_STACK_ATTRIBUTE __attribute__((section(".ccm")))
/* Stack depth, in words, of each work queue task. */
#define vkconfigWORK_QUEUE_STACK_SIZE 1024

/* Features. */
#define configUSE_MUTEXES             1
#define configUSE_RECURSIVE_MUTEXES   0
#define configUSE_COUNTING_SEMAPHORES 1
#define configUSE_TASK_NOTIFICATIONS  1
#define configUSE_TIMERS              0
#define configUSE_CO_ROUTINES         0
#define configQUEUE_REGISTRY_SIZE     0

/* Hooks. The idle hook puts the CPU to sleep with WFI. */
#define configUSE_IDLE_HOOK            1
#define configUSE_TICK_HOOK            0
#define configUSE_MALLOC_FAILED_HOOK   0
#define configCHECK_FOR_STACK_OVERFLOW 2

/* API functions compiled in. */
#define INCLUDE_vTaskDelay                1
#define INCLUDE_xTaskDelayUntil           1
#define INCLUDE_vTaskSuspend              1
#define INCLUDE_xTaskGetCurrentTaskHandle 1
#define INCLUDE_xTaskGetSchedulerState    1
#define INCLUDE_uxTaskGetStackHighWaterMark 1

/*
 * Interrupt priorities. The STM32F4 implements 4 priority bits. Interrupts at
 * priority 5 to 15 may call FromISR APIs; 0 to 4 are never masked by the
 * kernel and must not call FreeRTOS (the microsecond clock overflow uses one).
 */
#define configPRIO_BITS                              4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY      15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configKERNEL_INTERRUPT_PRIORITY      (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/* A failed assertion stops the CPU where a debugger can see it. */
#define configASSERT(x)              \
    if ((x) == 0) {                  \
        __asm volatile("cpsid i");   \
        for (;;) {                   \
        }                            \
    }

/* The vector table names the port's handlers directly. */
#define vPortSVCHandler    SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

#endif /* FREERTOS_CONFIG_H */
