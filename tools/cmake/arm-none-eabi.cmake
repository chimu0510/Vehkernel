# arm-none-eabi-gcc 交叉編譯工具鏈（STM32F407，Cortex-M4F）。
# 工具鏈需在 PATH 中，或以 -DARM_TOOLCHAIN_DIR=<.../bin> 指定。

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(ARM_TOOLCHAIN_DIR "" CACHE PATH "Directory holding arm-none-eabi-gcc (empty: search PATH)")
if(ARM_TOOLCHAIN_DIR)
    set(_hint HINTS "${ARM_TOOLCHAIN_DIR}")
endif()
find_program(ARM_GCC arm-none-eabi-gcc ${_hint} REQUIRED)
get_filename_component(_bin "${ARM_GCC}" DIRECTORY)

set(CMAKE_C_COMPILER   "${_bin}/arm-none-eabi-gcc")
set(CMAKE_ASM_COMPILER "${_bin}/arm-none-eabi-gcc")
set(CMAKE_OBJCOPY      "${_bin}/arm-none-eabi-objcopy" CACHE FILEPATH "")
set(CMAKE_SIZE         "${_bin}/arm-none-eabi-size" CACHE FILEPATH "")

# 交叉編譯時 try_compile 無法執行連結出來的程式，改成只編譯成靜態庫。
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(MCU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(CMAKE_C_FLAGS_INIT   "${MCU_FLAGS} -ffunction-sections -fdata-sections -fno-common")
set(CMAKE_ASM_FLAGS_INIT "${MCU_FLAGS} -x assembler-with-cpp")
set(CMAKE_EXE_LINKER_FLAGS_INIT
    "${MCU_FLAGS} --specs=nano.specs --specs=nosys.specs -Wl,--gc-sections -u _printf_float")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
