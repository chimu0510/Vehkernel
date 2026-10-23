# SPDX-License-Identifier: GPL-3.0-or-later
# Interface layer: HAL contracts (ADR-0009). Contracts are added as drivers need them.
vk_add_library(vk_hal LAYER interface INTERFACE)
target_include_directories(vk_hal INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
