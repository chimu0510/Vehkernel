# L1 unit tests on the development host (ADR-0018).
find_package(GTest CONFIG REQUIRED)
include(GoogleTest)

add_executable(vk_unit_tests smoke_test.cpp pin_test.cpp)
target_link_libraries(vk_unit_tests PRIVATE GTest::gtest_main vk_hal vk_warnings)
gtest_discover_tests(vk_unit_tests)
