# L1 unit tests on the development host (ADR-0018).
find_package(GTest CONFIG REQUIRED)
include(GoogleTest)

add_executable(vk_unit_tests smoke_test.cpp pin_test.cpp dma_test.cpp clock_test.cpp schema_test.cpp bus_test.cpp work_queue_test.cpp i2c_test.cpp rotation_test.cpp icm20948_test.cpp wire_test.cpp link_test.cpp link_work_item_test.cpp uart_test.cpp estimation_math_test.cpp mahony_test.cpp attitude_estimator_test.cpp parameters_test.cpp parameter_storage_test.cpp)
target_link_libraries(vk_unit_tests PRIVATE GTest::gtest_main vk_hal vk_msg vk_drivers vk_sim vk_platform vk_core vk_link vk_lib_estimation vk_estimation vk_params vk_warnings)
# Hardware-free logic of the STM32F4 platform layer, tested on the host.
target_include_directories(vk_unit_tests PRIVATE ${CMAKE_SOURCE_DIR}/platform/stm32f4/include)
gtest_discover_tests(vk_unit_tests)

# Cross-language check of the generated encoders: C++ encodes, Python compares.
add_executable(vk_wire_vectors wire_vectors.cpp)
target_link_libraries(vk_wire_vectors PRIVATE vk_msg_headers vk_warnings)
add_test(NAME wire_cross_language
  COMMAND ${Python3_EXECUTABLE} -m pytest -q -p no:cacheprovider ${CMAKE_CURRENT_SOURCE_DIR}/test_wire_cross_language.py)
set_tests_properties(wire_cross_language PROPERTIES ENVIRONMENT
  "VK_WIRE_VECTORS=$<TARGET_FILE:vk_wire_vectors>;VK_MSG_PYTHON_DIR=${VK_MSG_PYTHON_DIR}")

# Board defaults, checked against the C50C V1.1 golden board header.
add_executable(vk_board_defaults_tests board_defaults_test.cpp)
target_include_directories(vk_board_defaults_tests PRIVATE
  ${CMAKE_SOURCE_DIR}/tools/generators/tests/boards/c50c-v1_1/expected)
target_link_libraries(vk_board_defaults_tests PRIVATE GTest::gtest_main vk_hal vk_warnings)
gtest_discover_tests(vk_board_defaults_tests)

# Every golden board.hpp of the board generator tests must compile: one
# translation unit per header, each checking its pins are distinct.
file(GLOB golden_headers
  ${CMAKE_SOURCE_DIR}/tools/generators/tests/cases/*/expected/board.hpp
  ${CMAKE_SOURCE_DIR}/tools/generators/tests/boards/*/expected/board.hpp)
foreach(header IN LISTS golden_headers)
  get_filename_component(case_dir ${header} DIRECTORY)
  get_filename_component(case_dir ${case_dir} DIRECTORY)
  get_filename_component(case ${case_dir} NAME)
  set(source ${CMAKE_CURRENT_BINARY_DIR}/golden_${case}.cpp)
  file(WRITE ${source}
    "#include \"${header}\"\n"
    "static_assert(vk::hal::distinct_pins<vk::board::pins::all>);\n")
  add_library(vk_golden_${case} OBJECT ${source})
  target_link_libraries(vk_golden_${case} PRIVATE vk_hal vk_warnings)
  set_target_properties(vk_golden_${case} PROPERTIES CXX_CLANG_TIDY "")
endforeach()

# Every golden output of the schema generator tests must compile: each case's
# registry source, which includes all of its headers.
file(GLOB schema_cases LIST_DIRECTORIES true ${CMAKE_SOURCE_DIR}/tools/generators/tests/schema/cases/valid_*)
foreach(case_dir IN LISTS schema_cases)
  get_filename_component(case ${case_dir} NAME)
  add_library(vk_golden_schema_${case} OBJECT ${case_dir}/expected/vk/msg/registry.cpp)
  target_include_directories(vk_golden_schema_${case} PRIVATE ${case_dir}/expected)
  target_link_libraries(vk_golden_schema_${case} PRIVATE vk_core vk_warnings)
  set_target_properties(vk_golden_schema_${case} PROPERTIES CXX_CLANG_TIDY "")
endforeach()

# Every golden output of the parameter generator tests must compile: each
# case's parameter table source, which includes its header.
file(GLOB params_cases LIST_DIRECTORIES true ${CMAKE_SOURCE_DIR}/tools/generators/tests/params/cases/valid_*)
foreach(case_dir IN LISTS params_cases)
  get_filename_component(case ${case_dir} NAME)
  add_library(vk_golden_params_${case} OBJECT ${case_dir}/expected/vk/params/params.cpp)
  target_include_directories(vk_golden_params_${case} PRIVATE ${case_dir}/expected)
  target_link_libraries(vk_golden_params_${case} PRIVATE vk_core vk_warnings)
  set_target_properties(vk_golden_params_${case} PROPERTIES CXX_CLANG_TIDY "")
endforeach()
