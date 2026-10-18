# Module layer: estimators, control chain, vehicle models, Links, logging (ADR-0009).
vk_add_library(vk_modules LAYER module INTERFACE)
vk_target_link_libraries(vk_modules INTERFACE vk_drivers vk_hal vk_core vk_lib_estimation)

if(CONFIG_LINK)
  # The custom protocol (ADR-0002). Its protocol code uses neither the
  # transport nor FreeRTOS, so it is tested on the development host.
  vk_add_library(vk_link LAYER module STATIC
    link/src/cobs.cpp
    link/src/crc16.cpp
    link/src/frame.cpp
    link/src/session.cpp)
  target_include_directories(vk_link PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/link/include)
  vk_target_link_libraries(vk_link PUBLIC vk_msg_headers vk_core vk_hal)
  target_link_libraries(vk_link PRIVATE vk_warnings vk_cxx_subset)
  vk_target_link_libraries(vk_modules INTERFACE vk_link)
endif()
