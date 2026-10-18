add_subdirectory(unit)

# L3 SITL smoke test (ADR-0018): the IMU sample topic is published at the
# configured 200 Hz, and no work item overruns.
add_test(NAME sitl_smoke
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/sitl/check_report.py
    $<TARGET_FILE:vehkernel-sitl> --duration 5 --expect-rate imu_sample:0=200)

# Kconfig smoke test (ADR-0017): a debug overlay that sets an option whose
# dependencies are not met must fail configuration.
add_test(NAME kconfig_overlay_smoke
  COMMAND ${CMAKE_COMMAND} -DVK_SOURCE_DIR=${CMAKE_SOURCE_DIR}
    -DVK_BUILD_DIR=${CMAKE_CURRENT_BINARY_DIR}/kconfig_overlay_smoke -DVK_GENERATOR=${CMAKE_GENERATOR}
    -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/kconfig_overlay_smoke.cmake)
add_test(NAME kconfig
  COMMAND ${Python3_EXECUTABLE} -m pytest -q -p no:cacheprovider ${CMAKE_SOURCE_DIR}/tools/kconfig/tests)

# Size budget smoke test (ADR-0017): a firmware image too large for its Flash
# region must fail the link with the region and overage named.
add_test(NAME flash_budget_smoke
  COMMAND ${CMAKE_COMMAND} -DVK_SOURCE_DIR=${CMAKE_SOURCE_DIR}
    -DVK_BUILD_DIR=${CMAKE_CURRENT_BINARY_DIR}/flash_budget_smoke -DVK_GENERATOR=${CMAKE_GENERATOR}
    -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/flash_budget_smoke.cmake)

# Host tool tests (pytest), through each tool's command line.
add_test(NAME mcudb
  COMMAND ${Python3_EXECUTABLE} -m pytest -q -p no:cacheprovider ${CMAKE_SOURCE_DIR}/tools/mcudb/tests)
add_test(NAME board_generator
  COMMAND ${Python3_EXECUTABLE} -m pytest -q -p no:cacheprovider ${CMAKE_SOURCE_DIR}/tools/generators/tests/test_board_generator.py)
add_test(NAME bringup_runner
  COMMAND ${Python3_EXECUTABLE} -m pytest -q -p no:cacheprovider ${CMAKE_SOURCE_DIR}/tools/bringup/tests)
add_test(NAME schema_generator
  COMMAND ${Python3_EXECUTABLE} -m pytest -q -p no:cacheprovider ${CMAKE_SOURCE_DIR}/tools/generators/tests/schema)
add_test(NAME link_protocol
  COMMAND ${Python3_EXECUTABLE} -m pytest -q -p no:cacheprovider ${CMAKE_SOURCE_DIR}/tools/link/tests)
set_tests_properties(link_protocol PROPERTIES ENVIRONMENT
  "PYTHONPATH=${CMAKE_SOURCE_DIR}/tools/link")

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
