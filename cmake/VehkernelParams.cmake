# Parameter generator integration (ADR-0012, ADR-0017).
#
# Runs as a custom command after the Kconfig solve: every params.toml in the
# repository is checked, and the ones Kconfig compiles in become parameter
# groups, the parameter table and the metadata file. On a board, the board
# generator's defaults replace the definitions' defaults, and a board default
# out of range fails the build. The output lives in the
# build tree and is never committed.

# vk_generate_params() creates the vk_params target. Call it after
# vk_configure_kconfig().
function(vk_generate_params)
  set(out_dir ${CMAKE_BINARY_DIR}/generated/params)
  set(generator ${CMAKE_SOURCE_DIR}/tools/generators/generate_params.py)
  set(config ${CMAKE_BINARY_DIR}/generated/config/.config)
  # Every definition in the layers that may have parameters, at any depth.
  file(GLOB_RECURSE definitions RELATIVE ${CMAKE_SOURCE_DIR} CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/core/params.toml
    ${CMAKE_SOURCE_DIR}/drivers/params.toml
    ${CMAKE_SOURCE_DIR}/modules/params.toml
    ${CMAKE_SOURCE_DIR}/platform/params.toml)
  list(SORT definitions)
  file(GLOB generator_sources CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/tools/generators/vkparams/*.py
    ${CMAKE_SOURCE_DIR}/tools/generators/vkschema/schema.py
    ${CMAKE_SOURCE_DIR}/tools/generators/vkboard/diagnostics.py)

  # On a board, its defaults replace the definitions' (ADR-0012).
  set(board_args "")
  set(board_inputs "")
  if(VK_BOARD)
    set(board_args --board-defaults ${VK_BOARD_GENERATED_DIR}/board_defaults.json --board boards/${VK_BOARD}/board.toml)
    set(board_inputs ${VK_BOARD_GENERATED_DIR}/board_defaults.json)
  endif()

  set(outputs
    ${out_dir}/vk/params/params.hpp
    ${out_dir}/vk/params/params.cpp
    ${out_dir}/params.json)

  add_custom_command(
    OUTPUT ${outputs}
    COMMAND ${Python3_EXECUTABLE} -I ${generator} ${definitions} --config ${config} ${board_args}
      --output-dir ${out_dir}
    # The generator leaves unchanged files alone; touch them so the outputs
    # are newer than the inputs and the command does not run again.
    COMMAND ${CMAKE_COMMAND} -E touch ${outputs}
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    DEPENDS ${definitions} ${config} ${board_inputs} ${generator} ${generator_sources}
    COMMENT "Generating parameter groups"
    VERBATIM)

  add_custom_target(vk_params_generate DEPENDS ${outputs})

  # Host tools read the metadata file from here (ADR-0012).
  set(VK_PARAM_METADATA ${out_dir}/params.json PARENT_SCOPE)

  # The parameter runtime is part of core/, so the groups and the table sit
  # with it and every layer may read them.
  vk_add_library(vk_params LAYER core STATIC ${out_dir}/vk/params/params.cpp)
  target_include_directories(vk_params PUBLIC ${out_dir})
  add_dependencies(vk_params vk_params_generate)
  vk_target_link_libraries(vk_params PUBLIC vk_core vk_msg_headers)
  target_link_libraries(vk_params PRIVATE vk_warnings vk_cxx_subset)
endfunction()
