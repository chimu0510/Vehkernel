cmake_minimum_required(VERSION 3.8)
project(ugv_model_viewer)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Models come from the wheeltec_robot_urdf extension (extensions/, not in git;
# see extensions/README.md). Override with -DMODEL_URDF_DIR=... or urdf_dir:=...
get_filename_component(_default_urdf_dir
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../extensions/wheeltec_robot_urdf/urdf" ABSOLUTE)
set(MODEL_URDF_DIR "${_default_urdf_dir}" CACHE PATH "Directory holding the *.urdf files")
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/model_urdf_dir.txt" "${MODEL_URDF_DIR}\n")

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS scripts/model_viewer DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/model_urdf_dir.txt"
  DESTINATION share/${PROJECT_NAME})

ament_package()
