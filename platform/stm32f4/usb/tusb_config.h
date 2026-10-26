/* SPDX-License-Identifier: GPL-3.0-or-later */
/* TinyUSB configuration for the STM32F4 platform (spec 0010): the USB OTG FS
 * core as a full-speed host for one gamepad, on FreeRTOS. Only the XInput
 * class driver the platform supplies is used; TinyUSB's own class drivers
 * are left out. */
#ifndef VK_TUSB_CONFIG_H
#define VK_TUSB_CONFIG_H

#define CFG_TUSB_MCU OPT_MCU_STM32F4
#define CFG_TUSB_OS OPT_OS_FREERTOS
#define CFG_TUSB_DEBUG 0

#define CFG_TUH_ENABLED 1
#define CFG_TUH_MAX_SPEED OPT_MODE_FULL_SPEED
#define BOARD_TUH_RHPORT 0

#define CFG_TUH_ENUMERATION_BUFSIZE 256
#define CFG_TUH_DEVICE_MAX 1
#define CFG_TUH_HUB 0
#define CFG_TUH_ENDPOINT_MAX 8

#define CFG_TUH_HID 0
#define CFG_TUH_CDC 0
#define CFG_TUH_MSC 0
#define CFG_TUH_VENDOR 0

#define CFG_TUH_MEM_ALIGN __attribute__((aligned(4)))

#endif
