# SPDX-License-Identifier: GPL-3.0-or-later
# Board examples for C50C V1.1 (spec 0013, ADR-0025).

add_subdirectory(${CMAKE_SOURCE_DIR}/examples/support examples/support)

function(add_board_example name)
  add_executable(${name} ${ARGN})
  target_link_libraries(${name} PRIVATE
    example_support
    vk_platform
    vk_core
    vk_drivers
    vk_board
    vk_config
    vk_modules
    vk_params
    vk_warnings
    vk_cxx_subset)
  vk_firmware_image(${name})
endfunction()

add_board_example(example-01-led 01-led/main.cpp)
