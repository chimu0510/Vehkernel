// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Result block of the bring-up test build (ADR-0018, L4).
 *
 * The bring-up firmware fills one of these at the RAM symbol
 * vk_bringup_results; the bring-up runner finds the symbol in the ELF and
 * reads the block over SWD. Interim: once a Link exists, results move to it.
 * Bump VK_BRINGUP_VERSION whenever the layout changes; the runner checks it.
 */
#ifndef VK_BRINGUP_RESULTS_H
#define VK_BRINGUP_RESULTS_H

#include <stdint.h>

#define VK_BRINGUP_MAGIC     0x55424B56u /* "VKBU" in little-endian memory */
#define VK_BRINGUP_VERSION   1u
#define VK_BRINGUP_MAX_CASES 4u
#define VK_BRINGUP_VALUES    4u

enum vk_bringup_status {
    VK_BRINGUP_NOT_RUN = 0,
    VK_BRINGUP_PASS = 1,
    VK_BRINGUP_FAIL = 2,
};

enum vk_bringup_case_id {
    /* value[0] = WHO_AM_I read, value[1] = 1 if the transfer was acknowledged. */
    VK_BRINGUP_IMU_WHO_AM_I = 1,
    /* value[0] = mean read time [ns], value[1] = longest read time [ns],
     * value[2] = achievable read rate [Hz], value[3] = failed reads. */
    VK_BRINGUP_IMU_READ_RATE = 2,
};

struct vk_bringup_case {
    uint32_t id;     /* enum vk_bringup_case_id */
    uint32_t status; /* enum vk_bringup_status */
    int32_t value[VK_BRINGUP_VALUES];
};

struct vk_bringup_results {
    uint32_t magic;
    uint32_t version;
    uint32_t case_count;
    uint32_t complete; /* 1 once every case has run */
    struct vk_bringup_case cases[VK_BRINGUP_MAX_CASES];
};

#endif /* VK_BRINGUP_RESULTS_H */
