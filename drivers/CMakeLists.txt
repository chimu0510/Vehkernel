# SPDX-License-Identifier: GPL-3.0-or-later
# Driver layer: device drivers (ADR-0009). Drivers depend only on the
# interface layer and core/, never on the platform layer. Kconfig decides
# which drivers are built (ADR-0017).
set(sources "")
set(includes "")
if(CONFIG_DRIVER_ICM20948)
  list(APPEND sources icm20948/src/icm20948.cpp)
  list(APPEND includes ${CMAKE_CURRENT_SOURCE_DIR}/icm20948/include)
endif()
if(CONFIG_DRIVER_SSD1306)
  list(APPEND sources ssd1306/src/ssd1306.cpp)
  list(APPEND includes ${CMAKE_CURRENT_SOURCE_DIR}/ssd1306/include)
endif()

if(sources)
  vk_add_library(vk_drivers LAYER driver STATIC ${sources})
  target_include_directories(vk_drivers PUBLIC ${includes})
  vk_target_link_libraries(vk_drivers PUBLIC vk_hal vk_core vk_msg_headers vk_lib_estimation)
  target_link_libraries(vk_drivers PRIVATE vk_warnings vk_cxx_subset)
else()
  vk_add_library(vk_drivers LAYER driver INTERFACE)
  vk_target_link_libraries(vk_drivers INTERFACE vk_hal vk_core vk_msg_headers vk_lib_estimation)
endif()
