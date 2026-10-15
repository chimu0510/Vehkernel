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

# Algorithmic core static library for host tests and educational inspection
set(COMMON_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../examples/common")
add_library(ugv_algorithms_c STATIC
    ${COMMON_DIR}/kinematics.c
    ${COMMON_DIR}/pid.c
    ${COMMON_DIR}/control_arbitration.c
)
target_include_directories(ugv_algorithms_c PUBLIC
    include
    c
    ${COMMON_DIR}
)

# Decoupled unit tests
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    enable_testing()

    # 1. Pure C framing & CRC tests (Layer 2)
    add_executable(test_frame tests/test_frame.c)
    target_link_libraries(test_frame PRIVATE ugv_protocol_c)
    add_test(NAME test_frame COMMAND test_frame)

    # 2. Pure C message serialization tests (Layer 3)
    add_executable(test_protocol_c tests/test_protocol_c.c)
    target_link_libraries(test_protocol_c PRIVATE ugv_protocol_c)
    add_test(NAME test_protocol_c COMMAND test_protocol_c)

    # 3. C++ streaming parser & API tests (Layer 4)
    add_executable(test_protocol_cpp tests/test_protocol_cpp.cpp)
    target_link_libraries(test_protocol_cpp PRIVATE ugv_protocol_cpp)
    add_test(NAME test_protocol_cpp COMMAND test_protocol_cpp)

    # 4. Differential Kinematics & Heading Mapping test
    add_executable(test_kinematics tests/test_kinematics.c)
    target_link_libraries(test_kinematics PRIVATE ugv_algorithms_c)
    add_test(NAME test_kinematics COMMAND test_kinematics)

    # 5. PID speed controller & anti-windup test
    add_executable(test_pid tests/test_pid.c)
    target_link_libraries(test_pid PRIVATE ugv_algorithms_c)
    add_test(NAME test_pid COMMAND test_pid)

    # 6. Priority arbitration & failsafe ramp test
    add_executable(test_arbitration tests/test_arbitration.c)
    target_link_libraries(test_arbitration PRIVATE ugv_algorithms_c)
    add_test(NAME test_arbitration COMMAND test_arbitration)
endif()
