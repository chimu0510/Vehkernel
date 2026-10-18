# SITL platform. FreeRTOS POSIX port and simulated peripherals come later (ADR-0018).
vk_add_library(vk_platform LAYER platform INTERFACE)
vk_target_link_libraries(vk_platform INTERFACE vk_hal vk_core)
