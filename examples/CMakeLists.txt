# C50C 控制板範例。每個範例是一個獨立的韌體（.elf/.bin/.hex）。
#
#   cmake -S examples -B examples/build -G Ninja
#   cmake --build examples/build
#
cmake_minimum_required(VERSION 3.20)

set(REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}/..")
if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${REPO_ROOT}/tools/cmake/arm-none-eabi.cmake")
endif()
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE MinSizeRel CACHE STRING "" FORCE)
endif()

project(c50c_examples C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(HAL_DIR    "${REPO_ROOT}/third_party/stm32f4xx_hal_driver")
set(DEVICE_DIR "${REPO_ROOT}/third_party/cmsis_device_f4")
set(CORE_DIR   "${REPO_ROOT}/third_party/cmsis_core")
if(NOT EXISTS "${HAL_DIR}/Inc/stm32f4xx_hal.h")
    message(FATAL_ERROR "HAL 不存在，請先執行: git submodule update --init --depth 1")
endif()

# ST HAL：只編譯 stm32f4xx_hal_conf.h 啟用的模組。
add_library(stm32f4_hal STATIC
    ${HAL_DIR}/Src/stm32f4xx_hal.c
    ${HAL_DIR}/Src/stm32f4xx_hal_adc.c
    ${HAL_DIR}/Src/stm32f4xx_hal_adc_ex.c
    ${HAL_DIR}/Src/stm32f4xx_hal_cortex.c
    ${HAL_DIR}/Src/stm32f4xx_hal_dma.c
    ${HAL_DIR}/Src/stm32f4xx_hal_dma_ex.c
    ${HAL_DIR}/Src/stm32f4xx_hal_exti.c
    ${HAL_DIR}/Src/stm32f4xx_hal_flash.c
    ${HAL_DIR}/Src/stm32f4xx_hal_flash_ex.c
    ${HAL_DIR}/Src/stm32f4xx_hal_gpio.c
    ${HAL_DIR}/Src/stm32f4xx_hal_pwr.c
    ${HAL_DIR}/Src/stm32f4xx_hal_pwr_ex.c
    ${HAL_DIR}/Src/stm32f4xx_hal_rcc.c
    ${HAL_DIR}/Src/stm32f4xx_hal_rcc_ex.c
    ${HAL_DIR}/Src/stm32f4xx_hal_uart.c
)
target_include_directories(stm32f4_hal PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/common      # stm32f4xx_hal_conf.h
    ${HAL_DIR}/Inc
    ${DEVICE_DIR}/Include
    ${CORE_DIR}/Include
)
target_compile_definitions(stm32f4_hal PUBLIC STM32F407xx USE_HAL_DRIVER HSE_VALUE=8000000U)
# 第三方程式碼的警告不是我們要處理的。
target_compile_options(stm32f4_hal PRIVATE -w)

# 範例共用的板級支援。
add_library(board STATIC
    common/board.c
    common/board_id.c
    common/console.c
    common/report.c
    common/soft_i2c.c
)
target_include_directories(board PUBLIC common)
target_link_libraries(board PUBLIC stm32f4_hal)
target_compile_options(board PUBLIC -Wall -Wextra -Wshadow -Wdouble-promotion)

# 啟動檔與中斷處理必須直接連結進每個韌體：放在靜態庫裡時，強符號的
# SysTick_Handler 等不會被拉進來，連結器會改用啟動檔裡的弱符號（無窮迴圈）。
add_library(board_runtime OBJECT
    ${DEVICE_DIR}/Source/Templates/gcc/startup_stm32f407xx.s
    ${DEVICE_DIR}/Source/Templates/system_stm32f4xx.c
    common/stm32f4xx_it.c
)
target_link_libraries(board_runtime PUBLIC board)

set(LINKER_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/common/STM32F407VETx_FLASH.ld")

# c50c_example(<目錄名>)：以 <目錄名>/*.c 建立一個韌體，並輸出 .bin/.hex 與大小。
function(c50c_example name)
    file(GLOB sources CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/${name}/*.c")
    add_executable(${name} ${sources})
    set_target_properties(${name} PROPERTIES SUFFIX ".elf")
    target_link_libraries(${name} PRIVATE board_runtime board m)
    target_link_options(${name} PRIVATE
        -T${LINKER_SCRIPT}
        -Wl,-Map=${name}.map,--cref
        -Wl,--print-memory-usage)
    set_property(TARGET ${name} APPEND PROPERTY LINK_DEPENDS ${LINKER_SCRIPT})
    add_custom_command(TARGET ${name} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${name}> ${name}.bin
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${name}> ${name}.hex
        COMMENT "${name}: .bin/.hex")
endfunction()

c50c_example(01_board_info)
c50c_example(02_led)
c50c_example(03_key_buzzer)
c50c_example(04_uart)
c50c_example(05_imu_icm20948)
c50c_example(06_battery_adc)
