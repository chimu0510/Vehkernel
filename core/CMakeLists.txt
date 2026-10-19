# Bus, Work Queues, parameter runtime and timing monitor (ADR-0008, ADR-0012, ADR-0015).
vk_add_library(vk_core LAYER core STATIC
  src/critical_section.cpp
  src/parameter_storage.cpp
  src/parameters.cpp
  src/scheduler.cpp
  src/timing_monitor.cpp
  src/work_queue.cpp)
target_include_directories(vk_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
vk_target_link_libraries(vk_core PUBLIC vk_hal vk_msg_headers freertos_kernel)
target_link_libraries(vk_core PRIVATE vk_warnings vk_cxx_subset)
