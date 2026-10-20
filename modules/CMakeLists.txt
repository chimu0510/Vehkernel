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

if(CONFIG_LINK)
  # The custom protocol and the Link work item (ADR-0002). The protocol code
  # uses neither a transport nor FreeRTOS, so it is tested on the development
  # host; the Link is tested there over a fake transport.
  vk_add_library(vk_link LAYER module STATIC
    link/src/cobs.cpp
    link/src/crc16.cpp
    link/src/default_topic_map.cpp
    link/src/frame.cpp
    link/src/link.cpp
    link/src/session.cpp)
  target_include_directories(vk_link PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/link/include)
  vk_target_link_libraries(vk_link PUBLIC vk_msg_headers vk_core vk_hal)
  target_link_libraries(vk_link PRIVATE vk_warnings vk_cxx_subset)
  vk_target_link_libraries(vk_modules INTERFACE vk_link)
endif()

if(CONFIG_ESTIMATOR_MAHONY)
  # Estimators (ADR-0014): work items over the math library in lib/estimation.
  vk_add_library(vk_estimation LAYER module STATIC estimation/src/attitude_estimator.cpp)
  target_include_directories(vk_estimation PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/estimation/include)
  vk_target_link_libraries(vk_estimation PUBLIC vk_msg_headers vk_core vk_params vk_lib_estimation)
  target_link_libraries(vk_estimation PRIVATE vk_warnings vk_cxx_subset)
  vk_target_link_libraries(vk_modules INTERFACE vk_estimation)
endif()
