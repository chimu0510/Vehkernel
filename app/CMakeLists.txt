# Entry points: the firmware image for STM32 boards, or the SITL executable.
if(VK_PLATFORM STREQUAL "stm32f4")
  add_executable(vehkernel firmware_main.cpp)
  target_link_libraries(vehkernel PRIVATE vk_platform vk_board vk_modules vk_warnings vk_cxx_subset)
  set_target_properties(vehkernel PROPERTIES SUFFIX .elf)
  target_link_options(vehkernel PRIVATE -Wl,-Map=$<TARGET_FILE_DIR:vehkernel>/vehkernel.map)
  add_custom_command(TARGET vehkernel POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:vehkernel> $<TARGET_FILE_DIR:vehkernel>/vehkernel.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:vehkernel> $<TARGET_FILE_DIR:vehkernel>/vehkernel.hex
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:vehkernel>
    VERBATIM)
else()
  add_executable(vehkernel-sitl sitl_main.cpp)
  target_link_libraries(vehkernel-sitl PRIVATE vk_platform vk_modules vk_warnings)
endif()
