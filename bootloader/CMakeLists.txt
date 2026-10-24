# SPDX-License-Identifier: GPL-3.0-or-later
# The Vehkernel bootloader (ADR-0023). Its core is header-only and needs only
# the app descriptor's layout from core, not core's FreeRTOS-based runtime,
# so it adds core's headers rather than linking vk_core.
vk_add_library(vk_bootloader LAYER core INTERFACE)
target_include_directories(vk_bootloader INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/core/include)

if(VK_PLATFORM STREQUAL "sitl")
  # The SITL bootloader on a pty, for CI to flash with uploader.py.
  add_executable(vehkernel-sitl-bootloader sitl/sitl_bootloader.cpp)
  target_link_libraries(vehkernel-sitl-bootloader PRIVATE vk_bootloader vk_warnings)
endif()

if(VK_PLATFORM STREQUAL "stm32f4" AND CONFIG_BOARD_BOOTLOADER_VEHKERNEL)
  # The bootloader on the board, in the BOOT region (ADR-0020): bare metal,
  # on the platform's clock tree and internal Flash code. The link fails when
  # it outgrows the region (ADR-0017). The firmware's build packs its .bin
  # into vehkernel_with_bl.hex (app/CMakeLists.txt).
  set(platform_dir ${CMAKE_SOURCE_DIR}/platform/stm32f4)
  add_executable(vehkernel-bootloader
    stm32f4/stm32_bootloader.cpp
    stm32f4/vectors.c
    ${platform_dir}/src/clock_tree.cpp
    ${platform_dir}/src/internal_flash.cpp)
  target_include_directories(vehkernel-bootloader PRIVATE ${platform_dir}/include)
  target_link_libraries(vehkernel-bootloader PRIVATE vk_bootloader vk_board vk_hal st_ll_f4 vk_warnings vk_cxx_subset)
  set_target_properties(vehkernel-bootloader PROPERTIES SUFFIX .elf)
  set(linker_script ${CMAKE_CURRENT_SOURCE_DIR}/stm32f4/bootloader.ld)
  target_link_options(vehkernel-bootloader PRIVATE
    -T${linker_script} -L${VK_BOARD_GENERATED_DIR}
    -Wl,-Map=$<TARGET_FILE_DIR:vehkernel-bootloader>/vehkernel-bootloader.map -Wl,--print-memory-usage)
  set_property(TARGET vehkernel-bootloader APPEND PROPERTY LINK_DEPENDS
    ${linker_script} ${VK_BOARD_GENERATED_DIR}/memory.ld)
  add_custom_command(TARGET vehkernel-bootloader POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:vehkernel-bootloader>
      $<TARGET_FILE_DIR:vehkernel-bootloader>/vehkernel-bootloader.bin
    VERBATIM)
endif()
