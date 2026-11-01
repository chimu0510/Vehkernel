# SPDX-License-Identifier: GPL-3.0-or-later
# Board examples for C50C V1.1 (spec 0013, ADR-0025).

include(${CMAKE_CURRENT_SOURCE_DIR}/sitl_targets.cmake)

add_subdirectory(${CMAKE_SOURCE_DIR}/examples/support examples/support)

if(VK_PLATFORM STREQUAL "stm32f4")
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
  add_board_example(example-02-user-key 02-user-key/main.cpp)
  add_board_example(example-03-oled 03-oled/main.cpp)
  add_board_example(example-04-work-queues 04-work-queues/main.cpp)
  add_board_example(example-05-bus 05-bus/main.cpp)
elseif(VK_PLATFORM STREQUAL "sitl")
  function(add_sitl_example dir)
    set(name example-${dir})
    add_executable(${name} ${dir}/main.cpp)
    target_link_libraries(${name} PRIVATE
      example_support
      vk_platform
      vk_core
      vk_drivers
      vk_config
      vk_modules
      vk_params
      vk_sim
      vk_warnings)

    string(REPLACE "-" "_" test_name "example-${dir}")
    add_test(NAME ${test_name}
      COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/test/sitl/check_examples.py
        $<TARGET_FILE:${name}>)
  endfunction()

  foreach(ex IN LISTS VK_SITL_EXAMPLES)
    add_sitl_example(${ex})
  endforeach()
endif()
