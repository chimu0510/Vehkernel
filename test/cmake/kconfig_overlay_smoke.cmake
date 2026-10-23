# SPDX-License-Identifier: GPL-3.0-or-later
# Configures SITL with a debug overlay enabling CONFIG_DRIVER_CAN, which
# depends on BOARD_HAS_CAN, and expects configuration to fail.
execute_process(
  COMMAND ${CMAKE_COMMAND} -S ${VK_SOURCE_DIR} -B ${VK_BUILD_DIR} -G ${VK_GENERATOR}
    -DVK_PLATFORM=sitl -DVK_DEBUG_OVERLAY=${VK_SOURCE_DIR}/test/cmake/bad_overlay.config
  RESULT_VARIABLE result OUTPUT_VARIABLE out ERROR_VARIABLE out)
if(result EQUAL 0)
  message(FATAL_ERROR "an overlay violating a dependency was accepted")
endif()
# CMake wraps long messages, so compare with whitespace collapsed.
string(REGEX REPLACE "[ \t\r\n]+" " " flat "${out}")
if(NOT flat MATCHES "bad_overlay.config:2: error: CONFIG_DRIVER_CAN=y did not take effect .* depends on BOARD_HAS_CAN")
  message(FATAL_ERROR "configuration failed, but not on the overlay:\n${out}")
endif()
message(STATUS "invalid overlay rejected as expected")
