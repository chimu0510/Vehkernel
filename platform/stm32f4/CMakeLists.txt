if(NOT VK_BOARD)
  message(FATAL_ERROR "STM32 builds need VK_BOARD; use a CMake preset")
endif()
vk_generate_board(${VK_BOARD})

# STM32F4 platform. An OBJECT library so the vector table is always linked in.
vk_add_library(vk_platform LAYER platform OBJECT startup_stm32f407.c)
vk_target_link_libraries(vk_platform PUBLIC vk_hal vk_core vk_warnings)

set(VK_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/stm32f407vet6.ld)
target_link_options(vk_platform INTERFACE -T${VK_LINKER_SCRIPT})
set_property(TARGET vk_platform APPEND PROPERTY INTERFACE_LINK_DEPENDS ${VK_LINKER_SCRIPT})
