# SPDX-License-Identifier: GPL-3.0-or-later
add_library(example_support INTERFACE)
target_include_directories(example_support INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(example_support INTERFACE vk_platform vk_core)
