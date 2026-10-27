# SPDX-License-Identifier: GPL-3.0-or-later
add_subdirectory(unit)

# L3 SITL smoke test (ADR-0018): the IMU sample topic is published at the
# configured 200 Hz, and no work item overruns. With the UDP Link, overruns
# are judged over a check window after start-up, by the MCU's own timing.
set(sitl_smoke_link_args)
if(CONFIG_LINK_UDP)
  set(sitl_smoke_link_args --msg-dir ${VK_MSG_PYTHON_DIR})
endif()
add_test(NAME sitl_smoke
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/sitl/check_report.py
    $<TARGET_FILE:vehkernel-sitl> --duration 5 ${sitl_smoke_link_args} --expect-rate imu_sample:0=200)

# L3 SITL Link test (spec 0003): the Link monitor connects to SITL over UDP,
# the handshake succeeds, imu_sample arrives at 200 Hz and no frame is lost.
if(CONFIG_LINK_UDP)
  add_test(NAME sitl_link
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/sitl/check_link.py
      $<TARGET_FILE:vehkernel-sitl> ${CMAKE_SOURCE_DIR}/tools/link/link_monitor.py
      --msg-dir ${VK_MSG_PYTHON_DIR} --duration 5 --expect-rate imu_sample:0=200)
endif()
# The same over the optional pty transport, which the monitor opens as a
# serial port; UDP stays SITL's default.
if(CONFIG_LINK_PTY)
  add_test(NAME sitl_link_pty
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/sitl/check_link.py
      $<TARGET_FILE:vehkernel-sitl> ${CMAKE_SOURCE_DIR}/tools/link/link_monitor.py --link pty
      --msg-dir ${VK_MSG_PYTHON_DIR} --duration 5 --expect-rate imu_sample:0=200)
endif()

# L3 SITL attitude tests (spec 0004): the estimate against the simulated
# IMU's true attitude, under each motion profile.
if(CONFIG_ESTIMATOR_MAHONY AND CONFIG_LINK_UDP)
  foreach(motion IN ITEMS tilt rotate shake)
    add_test(NAME sitl_attitude_${motion}
      COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/sitl/check_attitude.py
        $<TARGET_FILE:vehkernel-sitl> --msg-dir ${VK_MSG_PYTHON_DIR} --motion ${motion})
  endforeach()
endif()

# L3 SITL parameter test (spec 0005): list, read, set and refusals through the
# Link monitor's param command, and a gain changed while the estimator runs.
if(CONFIG_ESTIMATOR_MAHONY AND CONFIG_LINK_UDP)
  add_test(NAME sitl_params
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/sitl/check_params.py
      $<TARGET_FILE:vehkernel-sitl> ${CMAKE_SOURCE_DIR}/tools/link/link_monitor.py
      --msg-dir ${VK_MSG_PYTHON_DIR} --param-meta ${VK_PARAM_METADATA})
endif()

# L3 SITL arming test (spec 0006): arming refused for each check in turn,
# actuator power settling, the emergency stop and a skipped power check, then
# arming accepted.
if(CONFIG_LINK_UDP)
  add_test(NAME sitl_arming
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/sitl/check_arming.py
      $<TARGET_FILE:vehkernel-sitl> ${CMAKE_SOURCE_DIR}/tools/link/link_monitor.py
      --msg-dir ${VK_MSG_PYTHON_DIR} --param-meta ${VK_PARAM_METADATA})
endif()

# L3 SITL bootloader test (spec 0008): ArduPilot's uploader.py flashes the
# SITL bootloader, which refuses another board's image, stays after a cut-off
# upload and follows the boot request.
add_test(NAME sitl_bootloader
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/sitl/check_bootloader.py
    $<TARGET_FILE:vehkernel-sitl-bootloader>)

# L3 SITL reboot test (spec 0008): a reboot refused while armed, and SITL
# ending with the bootloader or firmware exit status once one is accepted.
if(CONFIG_LINK_UDP)
  add_test(NAME sitl_reboot
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/sitl/check_reboot.py
      $<TARGET_FILE:vehkernel-sitl> ${CMAKE_SOURCE_DIR}/tools/link/link_monitor.py
      --msg-dir ${VK_MSG_PYTHON_DIR} --param-meta ${VK_PARAM_METADATA})
endif()

# L3 SITL actuator test (spec 0006): direct-actuator setpoints turn the
# simulated wheels at 10 rad/s, and the setpoint timeout, actuator power loss,
# the emergency stop, a skipped power check and the idle disarm stop them.
if(CONFIG_ACTUATOR_CONTROL AND CONFIG_LINK_UDP)
  add_test(NAME sitl_actuators
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/sitl/check_actuators.py
      $<TARGET_FILE:vehkernel-sitl> --msg-dir ${VK_MSG_PYTHON_DIR})
endif()

# L3 SITL ground vehicle test (spec 0009): the simulated motors move the
# vehicle as its wheel kinematics say, the gyro feels its turns, and slip
# takes its share.
if(CONFIG_ACTUATOR_CONTROL AND CONFIG_VEHICLE_MODEL_WHEELED AND CONFIG_LINK_UDP)
  add_test(NAME sitl_vehicle
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/sitl/check_vehicle.py
      $<TARGET_FILE:vehkernel-sitl> --msg-dir ${VK_MSG_PYTHON_DIR})
endif()

# L3 SITL body velocity test (spec 0009): body velocity setpoints in TEST
# mode, allocated by the vehicle model, beside direct-actuator setpoints, and
# the Link monitor's drive-body.
if(CONFIG_CONTROL_CHAIN AND CONFIG_VEHICLE_MODEL_WHEELED AND CONFIG_LINK_UDP)
  add_test(NAME sitl_body_velocity
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/sitl/check_body_velocity.py
      $<TARGET_FILE:vehkernel-sitl> ${CMAKE_SOURCE_DIR}/tools/link/link_monitor.py --msg-dir ${VK_MSG_PYTHON_DIR})
endif()

# L3 SITL body motion test (spec 0009): the control chain against the true
# motion, a circle with and without slip and a twist beyond the wheels' reach.
if(CONFIG_CONTROL_CHAIN AND CONFIG_VEHICLE_MODEL_WHEELED AND CONFIG_LINK_UDP)
  add_test(NAME sitl_body_motion
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/sitl/check_body_motion.py
      $<TARGET_FILE:vehkernel-sitl> --msg-dir ${VK_MSG_PYTHON_DIR})
endif()

# L3 SITL odometry test (spec 0009): local odometry against the truth, and
# its heading from the attitude estimator when the wheels slip.
if(CONFIG_ESTIMATOR_LOCAL_ODOMETRY AND CONFIG_CONTROL_CHAIN AND CONFIG_VEHICLE_MODEL_WHEELED AND CONFIG_LINK_UDP)
  add_test(NAME sitl_odometry
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/sitl/check_odometry.py
      $<TARGET_FILE:vehkernel-sitl> --msg-dir ${VK_MSG_PYTHON_DIR})
endif()

# L3 SITL route test (spec 0009): forward 1 m and a square by body velocity,
# against the truth and the odometry.
if(CONFIG_ESTIMATOR_LOCAL_ODOMETRY AND CONFIG_CONTROL_CHAIN AND CONFIG_VEHICLE_MODEL_WHEELED AND CONFIG_LINK_UDP)
  add_test(NAME sitl_routes
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/sitl/check_routes.py
      $<TARGET_FILE:vehkernel-sitl> --msg-dir ${VK_MSG_PYTHON_DIR})
endif()

# L3 SITL remote control test (spec 0010): the simulated gamepad takes over
# in MANUAL, arms, drives and disarms, and the RC failsafe.
if(CONFIG_CONTROL_CHAIN AND CONFIG_VEHICLE_MODEL_WHEELED AND CONFIG_LINK_UDP)
  add_test(NAME sitl_remote_control
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/sitl/check_remote_control.py
      $<TARGET_FILE:vehkernel-sitl> --msg-dir ${VK_MSG_PYTHON_DIR})
endif()

# L3 SITL fault test (ADR-0005): a failed assertion and fail() while driving
# end SITL with every output stopped, and a watchdog or fault reset refuses
# arming.
if(CONFIG_ACTUATOR_CONTROL AND CONFIG_LINK_UDP)
  add_test(NAME sitl_fault
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/sitl/check_fault.py
      $<TARGET_FILE:vehkernel-sitl> --msg-dir ${VK_MSG_PYTHON_DIR})
endif()

# L3 SITL bring-up test (spec 0006): the motor-to-encoder case finds a
# reversed motor, and passes once MOTn_REV is set.
if(CONFIG_BRINGUP_MOTORS AND CONFIG_LINK_UDP)
  add_test(NAME sitl_bringup
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/sitl/check_bringup.py
      $<TARGET_FILE:vehkernel-sitl> ${CMAKE_SOURCE_DIR}/tools/link/link_monitor.py
      --msg-dir ${VK_MSG_PYTHON_DIR} --param-meta ${VK_PARAM_METADATA})
endif()

# L3 SITL display test (spec 0007): the status page on the simulated
# SSD1306, read back from the image SITL writes with --display.
if(CONFIG_LOCAL_DISPLAY)
  add_test(NAME sitl_display
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/sitl/check_display.py $<TARGET_FILE:vehkernel-sitl>)
endif()

# L3 SITL local menu test (spec 0007): the simulated user key opens the menu,
# edits and saves parameters, refuses arming while open and times out.
if(CONFIG_LOCAL_MENU AND CONFIG_LINK_UDP)
  add_test(NAME sitl_local_menu
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/sitl/check_local_menu.py $<TARGET_FILE:vehkernel-sitl>
      --msg-dir ${VK_MSG_PYTHON_DIR})
endif()

# Every SITL test judges timing on the host's clock, so none runs alongside
# another test under ctest -j: another SITL, or a test that configures and
# builds, contends for the host and stalls SITL's work queues.
get_property(_vk_sitl_tests DIRECTORY PROPERTY TESTS)
list(FILTER _vk_sitl_tests INCLUDE REGEX "^sitl_")
set_tests_properties(${_vk_sitl_tests} PROPERTIES RUN_SERIAL TRUE)

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

# License check (L0, ADR-0018): every file carries an SPDX-License-Identifier
# line or is listed in REUSE.toml. reuse asks git which files are tracked, so
# the checkout is trusted even when its owner differs from the container user.
add_test(NAME reuse_lint
  COMMAND ${Python3_EXECUTABLE} -m reuse --root ${CMAKE_SOURCE_DIR} lint)
set_tests_properties(reuse_lint PROPERTIES ENVIRONMENT
  "GIT_CONFIG_COUNT=1;GIT_CONFIG_KEY_0=safe.directory;GIT_CONFIG_VALUE_0=*")

# Host tool tests (pytest), through each tool's command line.
add_test(NAME socdb
  COMMAND ${Python3_EXECUTABLE} -m pytest -q -p no:cacheprovider ${CMAKE_SOURCE_DIR}/tools/socdb/tests)
add_test(NAME board_generator
  COMMAND ${Python3_EXECUTABLE} -m pytest -q -p no:cacheprovider ${CMAKE_SOURCE_DIR}/tools/generators/tests/test_board_generator.py)
add_test(NAME bringup_runner
  COMMAND ${Python3_EXECUTABLE} -m pytest -q -p no:cacheprovider ${CMAKE_SOURCE_DIR}/tools/bringup/tests)
add_test(NAME serial_flasher
  COMMAND ${Python3_EXECUTABLE} -m pytest -q -p no:cacheprovider ${CMAKE_SOURCE_DIR}/tools/flash/tests)
add_test(NAME firmware_finalize
  COMMAND ${Python3_EXECUTABLE} -m pytest -q -p no:cacheprovider ${CMAKE_SOURCE_DIR}/tools/firmware/tests)
add_test(NAME schema_generator
  COMMAND ${Python3_EXECUTABLE} -m pytest -q -p no:cacheprovider ${CMAKE_SOURCE_DIR}/tools/generators/tests/schema)
add_test(NAME params_generator
  COMMAND ${Python3_EXECUTABLE} -m pytest -q -p no:cacheprovider ${CMAKE_SOURCE_DIR}/tools/generators/tests/params)
add_test(NAME link_protocol
  COMMAND ${Python3_EXECUTABLE} -m pytest -q -p no:cacheprovider ${CMAKE_SOURCE_DIR}/tools/link/tests)
set_tests_properties(link_protocol PROPERTIES ENVIRONMENT
  "PYTHONPATH=${CMAKE_SOURCE_DIR}/tools/link")
add_test(NAME ulog
  COMMAND ${Python3_EXECUTABLE} -m pytest -q -p no:cacheprovider ${CMAKE_SOURCE_DIR}/tools/log/tests)
set_tests_properties(ulog PROPERTIES ENVIRONMENT
  "PYTHONPATH=${CMAKE_SOURCE_DIR}/tools/log;PYTHONDONTWRITEBYTECODE=1")

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
vk_add_layer_rule_test(platform lib allow)
vk_add_layer_rule_test(module platform reject)
vk_add_layer_rule_test(driver module reject)
vk_add_layer_rule_test(driver platform reject)
vk_add_layer_rule_test(core platform reject)
vk_add_layer_rule_test(interface core reject)
vk_add_layer_rule_test(lib core reject)
