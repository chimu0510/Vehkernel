cmake_minimum_required(VERSION 3.16)
project(ugv_protocol C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# C protocol static library
add_library(ugv_protocol_c STATIC
    include/ugv_protocol/frame.c
)
target_include_directories(ugv_protocol_c PUBLIC
    include
    c
)

# C++ protocol interface library
add_library(ugv_protocol_cpp INTERFACE)
target_include_directories(ugv_protocol_cpp INTERFACE
    include
    cpp
)
target_link_libraries(ugv_protocol_cpp INTERFACE ugv_protocol_c)

# Test executable
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    enable_testing()
    add_executable(test_codec tests/test_codec.cpp)
    target_link_libraries(test_codec PRIVATE ugv_protocol_cpp)
    add_test(NAME test_codec COMMAND test_codec)
endif()
