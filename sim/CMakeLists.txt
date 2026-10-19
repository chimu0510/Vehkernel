# Simulated peripherals: the simulated I2C bus and device register models,
# shared by SITL and the unit tests (ADR-0018). Part of the platform layer:
# drivers never see it, only the I2C contract.
vk_add_library(vk_sim LAYER platform STATIC
  src/i2c_bus.cpp
  src/icm20948.cpp
  src/motion.cpp
  src/nor_flash.cpp)
target_include_directories(vk_sim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
vk_target_link_libraries(vk_sim PUBLIC vk_hal vk_lib_estimation)
target_link_libraries(vk_sim PRIVATE vk_warnings)
