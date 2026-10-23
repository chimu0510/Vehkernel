# Entry points: the firmware image for STM32 boards, or the SITL executable.
if(VK_PLATFORM STREQUAL "stm32f4")
  if(CONFIG_BRINGUP_TESTS)
    # The bring-up test build replaces the application (ADR-0018).
    add_executable(vehkernel ${CMAKE_SOURCE_DIR}/test/hardware/bringup_main.cpp)
  else()
    add_executable(vehkernel firmware_main.cpp)
  endif()
  target_link_libraries(vehkernel PRIVATE vk_platform vk_core vk_drivers vk_board vk_config vk_modules vk_params vk_warnings vk_cxx_subset)
  set_target_properties(vehkernel PROPERTIES SUFFIX .elf)
  target_link_options(vehkernel PRIVATE
    -Wl,-Map=$<TARGET_FILE_DIR:vehkernel>/vehkernel.map -Wl,--print-memory-usage)
  if(VK_FILL_FLASH_KB)
    # Size budget smoke test only: make the image too large for its region.
    target_compile_definitions(vehkernel PRIVATE VK_FILL_FLASH_KB=${VK_FILL_FLASH_KB})
  endif()
  # The firmware .bin is the FIRMWARE region only, so flashing it never touches
  # the parameter sectors. finalize_image.py fills its app descriptor, in the
  # .bin and the .elf, and writes the .apj (ADR-0023); the .hex, holding the
  # boot stub and the firmware, is made from the finalized .elf.
  set(out $<TARGET_FILE_DIR:vehkernel>)
  add_custom_command(TARGET vehkernel POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary -R .boot_stub $<TARGET_FILE:vehkernel> ${out}/vehkernel.bin
    COMMAND ${Python3_EXECUTABLE} -I ${CMAKE_SOURCE_DIR}/tools/firmware/finalize_image.py
      --elf $<TARGET_FILE:vehkernel> --bin ${out}/vehkernel.bin
      --board-json ${VK_BOARD_GENERATED_DIR}/board.json --apj ${out}/vehkernel.apj
      --version ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR} --source-dir ${CMAKE_SOURCE_DIR}
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:vehkernel> ${out}/vehkernel.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary -j .boot_stub $<TARGET_FILE:vehkernel> ${out}/vehkernel-boot.bin
    VERBATIM)
else()
  add_executable(vehkernel-sitl sitl_main.cpp)
  target_link_libraries(vehkernel-sitl PRIVATE vk_platform vk_core vk_msg vk_drivers vk_config vk_modules vk_params vk_warnings)
endif()
