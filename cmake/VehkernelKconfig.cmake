# Kconfig integration (ADR-0017).
#
# Runs at configure time, after the board generator, because CMake itself
# needs the CONFIG_* values to decide what to build. Layers, in merge order:
#   1. MCU defaults (mcu/<mcu>/defconfig), or platform/sitl/defconfig for SITL
#   2. Board defaults (boards/<board>/defconfig)
#   3. Debug overlays: <build>/debug.config, written by the menuconfig target,
#      and never committed, then the files listed in VK_DEBUG_OVERLAY, such
#      as the committed test/hardware/bringup.config of the bring-up preset.

set(VK_DEBUG_OVERLAY "" CACHE STRING "Extra Kconfig fragments merged last, for local debugging")

function(vk_configure_kconfig)
  set(out_dir ${CMAKE_BINARY_DIR}/generated/config)
  set(local_overlay ${CMAKE_BINARY_DIR}/debug.config)
  if(NOT EXISTS ${local_overlay})
    file(WRITE ${local_overlay} "# Local debug overlay (ADR-0017). Edit by hand or with the menuconfig target.\n")
  endif()

  if(VK_PLATFORM STREQUAL "sitl")
    set(board_kconfig ${out_dir}/Kconfig.board)
    file(MAKE_DIRECTORY ${out_dir})
    file(WRITE ${board_kconfig} "# SITL has no board definition.\n")
    set(fragments ${CMAKE_SOURCE_DIR}/platform/sitl/defconfig)
  else()
    set(board_kconfig ${VK_BOARD_GENERATED_DIR}/Kconfig.board)
    set(fragments
      ${CMAKE_SOURCE_DIR}/mcu/${VK_BOARD_MCU}/defconfig
      ${CMAKE_SOURCE_DIR}/boards/${VK_BOARD}/defconfig)
  endif()
  list(APPEND fragments ${local_overlay} ${VK_DEBUG_OVERLAY})

  set(args --kconfig ${CMAKE_SOURCE_DIR}/Kconfig --board-kconfig ${board_kconfig} --output-dir ${out_dir})
  foreach(fragment IN LISTS fragments)
    list(APPEND args --fragment ${fragment})
  endforeach()
  set(tool ${CMAKE_SOURCE_DIR}/tools/kconfig/vkconfig.py)

  execute_process(
    COMMAND ${Python3_EXECUTABLE} -I ${tool} generate ${args}
    RESULT_VARIABLE result
    ERROR_VARIABLE diagnostics)
  if(NOT result EQUAL 0)
    message(FATAL_ERROR "Kconfig configuration is invalid:\n${diagnostics}")
  endif()

  file(GLOB_RECURSE kconfig_files ${CMAKE_SOURCE_DIR}/Kconfig ${CMAKE_SOURCE_DIR}/*/Kconfig)
  set_property(DIRECTORY ${CMAKE_SOURCE_DIR} APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    ${tool} ${kconfig_files} ${fragments})

  include(${out_dir}/kconfig.cmake)

  add_library(vk_config INTERFACE)
  target_include_directories(vk_config INTERFACE ${out_dir})

  add_custom_target(menuconfig
    COMMAND ${Python3_EXECUTABLE} -I ${tool} menuconfig ${args} --overlay ${local_overlay}
    USES_TERMINAL
    COMMENT "Edit the configuration; changes are saved to ${local_overlay}")

  # Make the solved CONFIG_* values visible to every directory.
  get_cmake_property(vars VARIABLES)
  foreach(var IN LISTS vars)
    if(var MATCHES "^CONFIG_")
      set(${var} "${${var}}" PARENT_SCOPE)
    endif()
  endforeach()
endfunction()
