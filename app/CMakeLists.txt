# Entry points: the firmware image for STM32 boards, or the SITL executable.
if(VK_PLATFORM STREQUAL "stm32f4")
  add_executable(vehkernel firmware_main.cpp)
  target_link_libraries(vehkernel PRIVATE vk_platform vk_core vk_board vk_config vk_modules vk_warnings vk_cxx_subset)
  set_target_properties(vehkernel PROPERTIES SUFFIX .elf)
  target_link_options(vehkernel PRIVATE
    -Wl,-Map=$<TARGET_FILE_DIR:vehkernel>/vehkernel.map -Wl,--print-memory-usage)
  if(VK_FILL_FLASH_KB)
    # Size budget smoke test only: make the image too large for its region.
    target_compile_definitions(vehkernel PRIVATE VK_FILL_FLASH_KB=${VK_FILL_FLASH_KB})
  endif()
  # The .hex holds the boot stub and the firmware. The .bin files are split so
  # that flashing firmware over DFU never touches the parameter sectors.
  set(out $<TARGET_FILE_DIR:vehkernel>)
  add_custom_command(TARGET vehkernel POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:vehkernel> ${out}/vehkernel.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary -R .boot_stub $<TARGET_FILE:vehkernel> ${out}/vehkernel.bin
    COMMAND ${CMAKE_OBJCOPY} -O binary -j .boot_stub $<TARGET_FILE:vehkernel> ${out}/vehkernel-boot.bin
    VERBATIM)
else()
  add_executable(vehkernel-sitl sitl_main.cpp)
  target_link_libraries(vehkernel-sitl PRIVATE vk_platform vk_core vk_config vk_modules vk_warnings)
endif()
