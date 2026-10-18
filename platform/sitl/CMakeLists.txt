# SITL platform on the FreeRTOS POSIX port (ADR-0018). An OBJECT library so
# the FreeRTOS hooks are always linked in.
vk_add_library(vk_platform LAYER platform OBJECT
  src/freertos_hooks.cpp
  src/microsecond_clock.cpp
  src/platform.cpp)
target_include_directories(vk_platform PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
vk_target_link_libraries(vk_platform PUBLIC vk_hal vk_core)
target_link_libraries(vk_platform PRIVATE freertos_kernel vk_warnings)
