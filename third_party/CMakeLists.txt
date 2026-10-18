# Third-party C code, pinned as git submodules at exact releases (ADR-0016).
#
# These targets have no layer, so the layering rules do not check them. Only
# core/ and the platform layer link FreeRTOS, and only the STM32F4 platform
# layer links the ST headers and LL drivers (ADR-0004, ADR-0009). Their
# include directories are SYSTEM so Vehkernel's -Werror does not apply to them.

find_package(Threads REQUIRED)

# FreeRTOS kernel. Each platform supplies FreeRTOSConfig.h in its freertos/ directory.
set(_freertos ${CMAKE_CURRENT_SOURCE_DIR}/freertos-kernel)
add_library(freertos_kernel STATIC
  ${_freertos}/tasks.c
  ${_freertos}/list.c
  ${_freertos}/queue.c
  ${_freertos}/timers.c
  ${_freertos}/event_groups.c
  ${_freertos}/stream_buffer.c)
target_include_directories(freertos_kernel SYSTEM PUBLIC
  ${_freertos}/include
  ${CMAKE_SOURCE_DIR}/platform/${VK_PLATFORM}/freertos)

if(VK_PLATFORM STREQUAL "stm32f4")
  target_sources(freertos_kernel PRIVATE ${_freertos}/portable/GCC/ARM_CM4F/port.c)
  target_include_directories(freertos_kernel SYSTEM PUBLIC ${_freertos}/portable/GCC/ARM_CM4F)
  # FreeRTOSConfig.h is C, so the board's system clock reaches it as a definition.
  string(JSON _sysclk_hz GET "${VK_BOARD_JSON}" clock sysclk_hz)
  target_compile_definitions(freertos_kernel PUBLIC VK_SYSCLK_HZ=${_sysclk_hz}u)

  # CMSIS core and STM32F4 device headers.
  add_library(st_cmsis_f4 INTERFACE)
  target_include_directories(st_cmsis_f4 SYSTEM INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/cmsis-core/CMSIS/Core/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/cmsis-device-f4/Include)
  string(JSON _hse_hz GET "${VK_BOARD_JSON}" clock hse_hz)
  target_compile_definitions(st_cmsis_f4 INTERFACE STM32F407xx HSE_VALUE=${_hse_hz}u)

  # ST LL drivers (ADR-0009). Only the modules the platform layer uses.
  set(_ll ${CMAKE_CURRENT_SOURCE_DIR}/stm32f4xx-hal-driver)
  add_library(st_ll_f4 STATIC
    ${_ll}/Src/stm32f4xx_ll_gpio.c
    ${_ll}/Src/stm32f4xx_ll_rcc.c
    ${_ll}/Src/stm32f4xx_ll_tim.c
    ${_ll}/Src/stm32f4xx_ll_utils.c)
  target_include_directories(st_ll_f4 SYSTEM PUBLIC ${_ll}/Inc)
  target_compile_definitions(st_ll_f4 PUBLIC USE_FULL_LL_DRIVER)
  target_link_libraries(st_ll_f4 PUBLIC st_cmsis_f4)
else()
  set(_posix ${_freertos}/portable/ThirdParty/GCC/Posix)
  target_sources(freertos_kernel PRIVATE ${_posix}/port.c ${_posix}/utils/wait_for_event.c)
  target_include_directories(freertos_kernel SYSTEM PUBLIC ${_posix} ${_posix}/utils)
  target_link_libraries(freertos_kernel PUBLIC Threads::Threads)
endif()
