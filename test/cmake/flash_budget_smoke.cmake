# SPDX-License-Identifier: GPL-3.0-or-later
# Configures the C50C V1.1 firmware with VK_FILL_FLASH_KB large enough to
# overflow its 448K FIRMWARE region, and expects the link to fail.
execute_process(
  COMMAND ${CMAKE_COMMAND} -S ${VK_SOURCE_DIR} -B ${VK_BUILD_DIR} -G ${VK_GENERATOR}
    --toolchain ${VK_SOURCE_DIR}/cmake/toolchain-arm-none-eabi.cmake
    -DCMAKE_BUILD_TYPE=MinSizeRel -DVK_PLATFORM=stm32f4 -DVK_BOARD=c50c-v1_1 -DVK_FILL_FLASH_KB=460
  RESULT_VARIABLE configure_result OUTPUT_VARIABLE out ERROR_VARIABLE out)
if(NOT configure_result EQUAL 0)
  message(FATAL_ERROR "configuring the oversized firmware failed:\n${out}")
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} --build ${VK_BUILD_DIR}
  RESULT_VARIABLE build_result OUTPUT_VARIABLE out ERROR_VARIABLE out)
if(build_result EQUAL 0)
  message(FATAL_ERROR "an image larger than the FIRMWARE region linked without error")
endif()
if(NOT out MATCHES "region `FIRMWARE' overflowed by [0-9]+ bytes")
  message(FATAL_ERROR "the link failed, but not with a FIRMWARE overflow:\n${out}")
endif()
message(STATUS "FIRMWARE overflow detected as expected")
