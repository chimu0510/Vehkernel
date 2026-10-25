# SPDX-License-Identifier: GPL-3.0-or-later
# Module layer: estimators, control chain, vehicle models, Links, logging (ADR-0009).
vk_add_library(vk_modules LAYER module INTERFACE)
vk_target_link_libraries(vk_modules INTERFACE vk_drivers vk_hal vk_core vk_lib_estimation)

# The commander (ADR-0005): modes, arming, arming checks and the emergency
# stop. Its state machine uses neither the bus nor FreeRTOS, so it is tested
# on the development host; the work item wraps it.
vk_add_library(vk_commander LAYER module STATIC
  commander/src/commander.cpp
  commander/src/commander_task.cpp)
target_include_directories(vk_commander PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/commander/include)
vk_target_link_libraries(vk_commander PUBLIC vk_msg_headers vk_core vk_params)
target_link_libraries(vk_commander PRIVATE vk_warnings vk_cxx_subset)
vk_target_link_libraries(vk_modules INTERFACE vk_commander)

# The vehicle model (ADR-0013): the vehicle description, read at boot, and
# the compiled-in models it selects from. No bus and no FreeRTOS, so it is
# tested on the development host.
vk_add_library(vk_vehicle_model LAYER module STATIC vehicle_model/src/vehicle_description.cpp)
if(CONFIG_VEHICLE_MODEL_WHEELED)
  target_sources(vk_vehicle_model PRIVATE vehicle_model/src/wheeled_model.cpp)
endif()
target_include_directories(vk_vehicle_model PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/vehicle_model/include)
vk_target_link_libraries(vk_vehicle_model PUBLIC vk_msg_headers vk_core vk_params vk_lib_estimation)
# Its header includes the compiled-in models, so the Kconfig header is public.
target_link_libraries(vk_vehicle_model PUBLIC vk_config PRIVATE vk_warnings vk_cxx_subset)
vk_target_link_libraries(vk_modules INTERFACE vk_vehicle_model)

if(CONFIG_ACTUATOR_CONTROL)
  # Actuator control (ADR-0013): the speed loops use neither the bus nor
  # FreeRTOS and are tested on the development host; the work item wraps them.
  vk_add_library(vk_actuators LAYER module STATIC
    actuators/src/actuator_control.cpp
    actuators/src/actuator_task.cpp
    actuators/src/bringup_case.cpp)
  target_include_directories(vk_actuators PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/actuators/include)
  vk_target_link_libraries(vk_actuators PUBLIC vk_msg_headers vk_core vk_hal vk_params)
  target_link_libraries(vk_actuators PRIVATE vk_warnings vk_cxx_subset)
  vk_target_link_libraries(vk_modules INTERFACE vk_actuators)
endif()

if(CONFIG_LINK)
  # The custom protocol and the Link work item (ADR-0002). The protocol code
  # uses neither a transport nor FreeRTOS, so it is tested on the development
  # host; the Link is tested there over a fake transport.
  vk_add_library(vk_link LAYER module STATIC
    link/src/cobs.cpp
    link/src/control_handler.cpp
    link/src/crc16.cpp
    link/src/default_topic_map.cpp
    link/src/frame.cpp
    link/src/frame_handler.cpp
    link/src/link.cpp
    link/src/parameter_handler.cpp
    link/src/session.cpp
    link/src/topic_map.cpp)
  if(CONFIG_PLATFORM_SITL)
    # The SITL-only frames; the board firmware never builds them (ADR-0018).
    target_sources(vk_link PRIVATE link/src/sim_handler.cpp)
  endif()
  target_include_directories(vk_link PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/link/include)
  vk_target_link_libraries(vk_link PUBLIC vk_msg_headers vk_core vk_hal)
  target_link_libraries(vk_link PRIVATE vk_warnings vk_cxx_subset)
  vk_target_link_libraries(vk_modules INTERFACE vk_link)
endif()

if(CONFIG_LOCAL_DISPLAY)
  # The local display (ADR-0022): page rendering uses neither the bus nor
  # FreeRTOS and is tested on the development host; the work item wraps it.
  vk_add_library(vk_local_display LAYER module STATIC
    local_display/src/font.cpp
    local_display/src/local_display.cpp
    local_display/src/local_menu.cpp
    local_display/src/pages.cpp)
  if(CONFIG_LOCAL_MENU)
    target_sources(vk_local_display PRIVATE
      local_display/src/local_menu_task.cpp
      local_display/src/user_key.cpp)
  endif()
  target_include_directories(vk_local_display PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/local_display/include)
  vk_target_link_libraries(vk_local_display PUBLIC vk_msg_headers vk_core vk_hal vk_drivers vk_lib_estimation vk_params)
  target_link_libraries(vk_local_display PRIVATE vk_warnings vk_cxx_subset)
  vk_target_link_libraries(vk_modules INTERFACE vk_local_display)
endif()

if(CONFIG_ESTIMATOR_MAHONY)
  # Estimators (ADR-0014): work items over the math library in lib/estimation.
  vk_add_library(vk_estimation LAYER module STATIC estimation/src/attitude_estimator.cpp)
  target_include_directories(vk_estimation PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/estimation/include)
  vk_target_link_libraries(vk_estimation PUBLIC vk_msg_headers vk_core vk_params vk_lib_estimation)
  target_link_libraries(vk_estimation PRIVATE vk_warnings vk_cxx_subset)
  vk_target_link_libraries(vk_modules INTERFACE vk_estimation)
endif()
