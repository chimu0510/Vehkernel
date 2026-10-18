cmake_minimum_required(VERSION 3.25)
project(Vehkernel LANGUAGES C CXX ASM)

set(VK_PLATFORM "" CACHE STRING "Platform layer to build: stm32f4 or sitl")
set_property(CACHE VK_PLATFORM PROPERTY STRINGS stm32f4 sitl)
set(VK_BOARD "" CACHE STRING "Board directory under boards/ (STM32 targets only)")
option(VK_BUILD_TESTS "Build and register host tests" OFF)
option(VK_CLANG_TIDY "Run clang-tidy on Vehkernel sources (L0, ADR-0018)" OFF)

if(NOT VK_PLATFORM MATCHES "^(stm32f4|sitl)$")
  message(FATAL_ERROR "VK_PLATFORM must be stm32f4 or sitl; use a CMake preset")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

find_package(Python3 COMPONENTS Interpreter REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
include(VehkernelLayers)
include(VehkernelBoard)
include(VehkernelKconfig)

# Warnings are errors for all Vehkernel code (L0, ADR-0018).
add_library(vk_warnings INTERFACE)
target_compile_options(vk_warnings INTERFACE -Wall -Wextra -Wshadow -Werror)

# C++20 subset for firmware code (ADR-0007).
add_library(vk_cxx_subset INTERFACE)
if(VK_PLATFORM STREQUAL "stm32f4")
  target_compile_options(vk_cxx_subset INTERFACE
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions -fno-rtti -fno-threadsafe-statics>)
endif()

if(VK_CLANG_TIDY)
  find_program(VK_CLANG_TIDY_EXE clang-tidy REQUIRED)
  set(CMAKE_CXX_CLANG_TIDY ${VK_CLANG_TIDY_EXE})
endif()

# Layers and shared parts (ADR-0009), bottom-up. The board generator and
# Kconfig run right after the interface layer exists, before anything that
# depends on their output.
add_subdirectory(hal)
if(VK_PLATFORM STREQUAL "stm32f4")
  if(NOT VK_BOARD)
    message(FATAL_ERROR "STM32 builds need VK_BOARD; use a CMake preset")
  endif()
  vk_generate_board(${VK_BOARD})
endif()
vk_configure_kconfig()
add_subdirectory(third_party)
add_subdirectory(core)
add_subdirectory(lib)
add_subdirectory(platform)
add_subdirectory(drivers)
add_subdirectory(modules)
add_subdirectory(app)

if(VK_BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()
