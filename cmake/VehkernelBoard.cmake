# Board generator integration (ADR-0011, ADR-0017).
#
# The generator runs at configure time rather than as a custom command: a
# wrong board definition must fail configuration, and Kconfig needs the
# board's symbols before the build starts. Editing any input re-runs the
# configure step automatically.

# vk_generate_board(<board>) creates the vk_board target holding board.hpp.
function(vk_generate_board board)
  set(board_file boards/${board}/board.toml)
  if(NOT EXISTS ${CMAKE_SOURCE_DIR}/${board_file})
    message(FATAL_ERROR "VK_BOARD '${board}' has no ${board_file}")
  endif()
  set(out_dir ${CMAKE_BINARY_DIR}/generated/board)
  set(generator ${CMAKE_SOURCE_DIR}/tools/generators/generate_board.py)

  execute_process(
    COMMAND ${Python3_EXECUTABLE} -I ${generator} ${board_file} --mcu-root mcu --output-dir ${out_dir}
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    RESULT_VARIABLE result
    ERROR_VARIABLE diagnostics)
  if(NOT result EQUAL 0)
    message(FATAL_ERROR "Board definition ${board_file} is invalid:\n${diagnostics}")
  endif()

  file(READ ${out_dir}/board.json board_json)
  string(JSON mcu GET "${board_json}" mcu)
  file(GLOB generator_sources ${CMAKE_SOURCE_DIR}/tools/generators/vkboard/*.py)
  set_property(DIRECTORY ${CMAKE_SOURCE_DIR} APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/${board_file}
    ${CMAKE_SOURCE_DIR}/mcu/${mcu}/mcu.json
    ${generator}
    ${generator_sources})

  set(VK_BOARD_GENERATED_DIR ${out_dir} PARENT_SCOPE)
  set(VK_BOARD_MCU ${mcu} PARENT_SCOPE)
  set(VK_BOARD_JSON "${board_json}" PARENT_SCOPE)
  vk_add_library(vk_board LAYER platform INTERFACE)
  target_include_directories(vk_board INTERFACE ${out_dir})
  vk_target_link_libraries(vk_board INTERFACE vk_hal)
endfunction()
