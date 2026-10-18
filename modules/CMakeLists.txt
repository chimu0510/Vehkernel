# Module layer: estimators, control chain, vehicle models, Links, logging (ADR-0009).
vk_add_library(vk_modules LAYER module INTERFACE)
vk_target_link_libraries(vk_modules INTERFACE vk_drivers vk_hal vk_core vk_lib_estimation)

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
