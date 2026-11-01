# SPDX-License-Identifier: GPL-3.0-or-later
add_library(example_support INTERFACE)
target_include_directories(example_support INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/${VK_PLATFORM}
)
target_link_libraries(example_support INTERFACE vk_platform vk_core vk_drivers vk_modules vk_params)

if(VK_PLATFORM STREQUAL "stm32f4")
  target_link_libraries(example_support INTERFACE vk_board)
elseif(VK_PLATFORM STREQUAL "sitl")
  target_link_libraries(example_support INTERFACE vk_sim)
endif()
