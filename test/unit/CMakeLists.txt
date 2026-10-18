# L1 unit tests on the development host (ADR-0018).
find_package(GTest CONFIG REQUIRED)
include(GoogleTest)

add_executable(vk_unit_tests smoke_test.cpp pin_test.cpp dma_test.cpp clock_test.cpp schema_test.cpp)
target_link_libraries(vk_unit_tests PRIVATE GTest::gtest_main vk_hal vk_msg vk_platform vk_core vk_warnings)
# Hardware-free logic of the STM32F4 platform layer, tested on the host.
target_include_directories(vk_unit_tests PRIVATE ${CMAKE_SOURCE_DIR}/platform/stm32f4/include)
gtest_discover_tests(vk_unit_tests)

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

# Every golden output of the schema generator tests must compile: one
# translation unit per case including all of its headers.
file(GLOB schema_cases LIST_DIRECTORIES true ${CMAKE_SOURCE_DIR}/tools/generators/tests/schema/cases/valid_*)
foreach(case_dir IN LISTS schema_cases)
  get_filename_component(case ${case_dir} NAME)
  file(GLOB_RECURSE case_headers ${case_dir}/expected/*.hpp)
  set(source ${CMAKE_CURRENT_BINARY_DIR}/golden_schema_${case}.cpp)
  set(text "")
  foreach(header IN LISTS case_headers)
    string(APPEND text "#include \"${header}\"\n")
  endforeach()
  file(WRITE ${source} "${text}")
  add_library(vk_golden_schema_${case} OBJECT ${source})
  target_link_libraries(vk_golden_schema_${case} PRIVATE vk_warnings)
  set_target_properties(vk_golden_schema_${case} PROPERTIES CXX_CLANG_TIDY "")
endforeach()
