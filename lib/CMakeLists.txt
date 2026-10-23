# SPDX-License-Identifier: GPL-3.0-or-later
# Estimation math components, independent of the bus and HAL (ADR-0014):
# math/ utilities, models/ process and measurement models, filters/ filter
# algorithms. Header-only, so code that does not use them costs nothing.
vk_add_library(vk_lib_estimation LAYER lib INTERFACE)
target_include_directories(vk_lib_estimation INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/estimation/math/include
  ${CMAKE_CURRENT_SOURCE_DIR}/estimation/models/include
  ${CMAKE_CURRENT_SOURCE_DIR}/estimation/filters/include)
