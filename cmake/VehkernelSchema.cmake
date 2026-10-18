# Schema generator integration (ADR-0006, ADR-0017).
#
# Runs as a custom command, so editing a .msg file regenerates its topics at
# build time. The generated headers live in the build tree and are never
# committed.

# vk_generate_schema() creates the vk_msg target holding the typed topics
# generated from schema/msg/*.msg, and vk_msg_headers with the message types
# alone. Call it before core/, which publishes topics of its own.
function(vk_generate_schema)
  set(out_dir ${CMAKE_BINARY_DIR}/generated/schema)
  set(generator ${CMAKE_SOURCE_DIR}/tools/generators/generate_schema.py)
  file(GLOB messages RELATIVE ${CMAKE_SOURCE_DIR} CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}/schema/msg/*.msg)
  list(SORT messages)
  file(GLOB generator_sources CONFIGURE_DEPENDS
    ${CMAKE_SOURCE_DIR}/tools/generators/vkschema/*.py
    ${CMAKE_SOURCE_DIR}/tools/generators/vkboard/diagnostics.py)

  set(headers ${out_dir}/vk/msg/registry.hpp ${out_dir}/vk/msg/registry.cpp)
  foreach(message IN LISTS messages)
    get_filename_component(name ${message} NAME_WE)
    list(APPEND headers ${out_dir}/vk/msg/${name}.hpp)
  endforeach()

  add_custom_command(
    OUTPUT ${headers}
    COMMAND ${Python3_EXECUTABLE} -I ${generator} ${messages} --output-dir ${out_dir}
    # The generator leaves unchanged files alone; touch them so the outputs
    # are newer than the inputs and the command does not run again.
    COMMAND ${CMAKE_COMMAND} -E touch ${headers}
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    DEPENDS ${messages} ${generator} ${generator_sources}
    COMMENT "Generating topics from the schema"
    VERBATIM)

  add_custom_target(vk_msg_generate DEPENDS ${headers})

  # Topics are part of the bus, so they sit with core/ and every layer may use
  # them. vk_msg_headers holds the message types; vk_msg adds the registry of
  # every topic instance, which needs core/.
  vk_add_library(vk_msg_headers LAYER core INTERFACE)
  target_include_directories(vk_msg_headers INTERFACE ${out_dir})
  add_dependencies(vk_msg_headers vk_msg_generate)

  vk_add_library(vk_msg LAYER core STATIC ${out_dir}/vk/msg/registry.cpp)
  vk_target_link_libraries(vk_msg PUBLIC vk_msg_headers vk_core)
  target_link_libraries(vk_msg PRIVATE vk_warnings vk_cxx_subset)
endfunction()
