add_subdirectory(unit)

# Host tool tests (pytest), through each tool's command line.
add_test(NAME mcudb
  COMMAND ${Python3_EXECUTABLE} -m pytest -q -p no:cacheprovider ${CMAKE_SOURCE_DIR}/tools/mcudb/tests)
add_test(NAME board_generator
  COMMAND ${Python3_EXECUTABLE} -m pytest -q -p no:cacheprovider ${CMAKE_SOURCE_DIR}/tools/generators/tests)

# Layer rules (ADR-0009): configure a scratch project that links one layer to
# another and check that the rule helper accepts or rejects it.
set(_vk_layer_project ${CMAKE_CURRENT_SOURCE_DIR}/cmake/layer_rules)
function(vk_add_layer_rule_test from to expect)
  set(name layer_rules.${from}_to_${to})
  add_test(NAME ${name}
    COMMAND ${CMAKE_COMMAND} -G ${CMAKE_GENERATOR}
      -S ${_vk_layer_project}
      -B ${CMAKE_CURRENT_BINARY_DIR}/layer_rules/${from}_to_${to}
      -DVK_SOURCE_DIR=${CMAKE_SOURCE_DIR} -DVK_FROM=${from} -DVK_TO=${to})
  if(expect STREQUAL "reject")
    set_tests_properties(${name} PROPERTIES WILL_FAIL TRUE)
  endif()
endfunction()

vk_add_layer_rule_test(module driver allow)
vk_add_layer_rule_test(module interface allow)
vk_add_layer_rule_test(driver core allow)
vk_add_layer_rule_test(platform core allow)
vk_add_layer_rule_test(core interface allow)
vk_add_layer_rule_test(module lib allow)
vk_add_layer_rule_test(module platform reject)
vk_add_layer_rule_test(driver module reject)
vk_add_layer_rule_test(driver platform reject)
vk_add_layer_rule_test(core platform reject)
vk_add_layer_rule_test(interface core reject)
vk_add_layer_rule_test(lib core reject)
