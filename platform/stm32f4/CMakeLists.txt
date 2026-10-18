# STM32F4 platform on ST LL (ADR-0009). An OBJECT library so the vector table
# and the FreeRTOS hooks are always linked in.
vk_add_library(vk_platform LAYER platform OBJECT
  startup_stm32f407.c
  boot_stub.c
  src/clock_tree.cpp
  src/freertos_hooks.cpp
  src/microsecond_clock.cpp
  src/platform.cpp)
target_include_directories(vk_platform PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
vk_target_link_libraries(vk_platform PUBLIC vk_hal vk_core vk_board)
target_link_libraries(vk_platform PUBLIC st_ll_f4)
target_link_libraries(vk_platform PRIVATE freertos_kernel vk_warnings vk_cxx_subset)

if(NOT EXISTS ${VK_BOARD_GENERATED_DIR}/memory.ld)
  message(FATAL_ERROR "Board ${VK_BOARD} has no Flash layout; STM32 builds need one (ADR-0020)")
endif()
set(VK_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/stm32f4.ld)
target_link_options(vk_platform INTERFACE -T${VK_LINKER_SCRIPT} -L${VK_BOARD_GENERATED_DIR})
set_property(TARGET vk_platform APPEND PROPERTY INTERFACE_LINK_DEPENDS
  ${VK_LINKER_SCRIPT} ${VK_BOARD_GENERATED_DIR}/memory.ld)
