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
