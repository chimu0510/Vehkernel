cmake_minimum_required(VERSION 3.8)
project(ugv_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch rviz config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
