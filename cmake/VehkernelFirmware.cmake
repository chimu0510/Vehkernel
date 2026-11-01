# SPDX-License-Identifier: GPL-3.0-or-later
# Firmware image finalization (ADR-0023, ADR-0025).

function(vk_firmware_image target)
  cmake_parse_arguments(PARSE_ARGV 1 arg "" "WITH_BL_BOOTLOADER;WITH_BL_HEX" "")

  set_target_properties(${target} PROPERTIES SUFFIX .elf)
  target_link_options(${target} PRIVATE
    -Wl,-Map=$<TARGET_FILE_DIR:${target}>/${target}.map -Wl,--print-memory-usage)

  set(out $<TARGET_FILE_DIR:${target}>)
  set(with_bl)
  if(arg_WITH_BL_BOOTLOADER AND arg_WITH_BL_HEX)
    set(with_bl --bootloader ${arg_WITH_BL_BOOTLOADER} --with-bl-hex ${arg_WITH_BL_HEX})
  endif()

  add_custom_command(TARGET ${target} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${target}> ${out}/${target}.bin
    COMMAND ${Python3_EXECUTABLE} -I ${CMAKE_SOURCE_DIR}/tools/firmware/finalize_image.py
      --elf $<TARGET_FILE:${target}> --bin ${out}/${target}.bin
      --board-json ${VK_BOARD_GENERATED_DIR}/board.json --apj ${out}/${target}.apj
      --version ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR} --source-dir ${CMAKE_SOURCE_DIR}
      ${with_bl}
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${target}> ${out}/${target}.hex
    VERBATIM)
endfunction()
