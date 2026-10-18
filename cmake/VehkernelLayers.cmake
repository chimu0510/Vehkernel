# Layer enforcement for ADR-0009.
#
# Every Vehkernel library belongs to one layer. vk_target_link_libraries()
# refuses any dependency the layering does not allow, so a violation fails at
# configure time instead of surfacing as a design drift in review.
#
#   module -> driver -> interface <- platform
#   core/ and lib/ sit outside the layering: every layer may use core and
#   lib, core uses only the interface layer, and lib uses nothing.
#
# Targets without a layer (third-party libraries, executables) are not checked.

set(VK_LAYERS module driver interface platform core lib)

set(_vk_allowed_module    module driver interface core lib)
set(_vk_allowed_driver    driver interface core lib)
set(_vk_allowed_interface interface)
set(_vk_allowed_platform  platform interface core lib)
set(_vk_allowed_core      core interface)
set(_vk_allowed_lib       lib)

# vk_add_library(<name> LAYER <layer> [STATIC|INTERFACE] [sources...])
function(vk_add_library name)
  cmake_parse_arguments(PARSE_ARGV 1 arg "" "LAYER" "")
  if(NOT arg_LAYER IN_LIST VK_LAYERS)
    message(FATAL_ERROR "vk_add_library(${name}): unknown layer '${arg_LAYER}', expected one of: ${VK_LAYERS}")
  endif()
  add_library(${name} ${arg_UNPARSED_ARGUMENTS})
  set_property(TARGET ${name} PROPERTY VK_LAYER ${arg_LAYER})
endfunction()

# vk_target_link_libraries(<target> <PUBLIC|PRIVATE|INTERFACE> <deps...>)
function(vk_target_link_libraries target scope)
  get_target_property(layer ${target} VK_LAYER)
  if(layer)
    foreach(dep IN LISTS ARGN)
      if(NOT TARGET ${dep})
        continue()
      endif()
      get_target_property(dep_layer ${dep} VK_LAYER)
      if(dep_layer AND NOT dep_layer IN_LIST _vk_allowed_${layer})
        message(FATAL_ERROR
          "Layer violation: ${target} (${layer}) must not depend on ${dep} (${dep_layer}). "
          "A ${layer} target may depend only on: ${_vk_allowed_${layer}} (ADR-0009)")
      endif()
    endforeach()
  endif()
  target_link_libraries(${target} ${scope} ${ARGN})
endfunction()
