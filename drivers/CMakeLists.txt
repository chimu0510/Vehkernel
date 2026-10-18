# Driver layer: device drivers (ADR-0009).
vk_add_library(vk_drivers LAYER driver INTERFACE)
vk_target_link_libraries(vk_drivers INTERFACE vk_hal vk_core vk_lib_estimation)
