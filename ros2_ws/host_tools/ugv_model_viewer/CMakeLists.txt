cmake_minimum_required(VERSION 3.8)
project(ugv_model_viewer)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# Models come from the wheeltec_robot_urdf extension (extensions/, not in git;
# see extensions/README.md). Override with -DMODEL_URDF_DIR=... or urdf_dir:=...
get_filename_component(_default_urdf_dir
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../extensions/wheeltec_robot_urdf/urdf" ABSOLUTE)
set(MODEL_URDF_DIR "${_default_urdf_dir}" CACHE PATH "Directory holding the *.urdf files")
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/model_urdf_dir.txt" "${MODEL_URDF_DIR}\n")

# RViz panel for browsing and switching models
set(CMAKE_AUTOMOC ON)
add_library(${PROJECT_NAME}_panel SHARED
  src/model_panel.cpp
  include/${PROJECT_NAME}/model_panel.hpp)
target_compile_features(${PROJECT_NAME}_panel PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(${PROJECT_NAME}_panel
  pluginlib::pluginlib
  rclcpp::rclcpp
  rviz_common::rviz_common
  ${std_msgs_TARGETS}
  Qt5::Widgets)
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)
install(TARGETS ${PROJECT_NAME}_panel LIBRARY DESTINATION lib)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS scripts/model_viewer DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/model_urdf_dir.txt"
  DESTINATION share/${PROJECT_NAME})

ament_package()
