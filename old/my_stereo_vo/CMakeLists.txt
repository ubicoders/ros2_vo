cmake_minimum_required(VERSION 3.8)
project(my_stereo_vo)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
cmake_policy(SET CMP0072 NEW)  # Prefer GLVND OpenGL libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(g2o REQUIRED)


# Library
add_library(${PROJECT_NAME}_lib
  src/stereo_visual_slam.cpp
  src/frontend.cpp
  src/frame.cpp
  src/feature.cpp
  src/pinhole_camera.cpp
  src/map.cpp
  src/map_point.cpp
  src/backend.cpp
)

target_link_libraries(${PROJECT_NAME}_lib
  g2o_core
  g2o_stuff
  g2o_types_sba
  g2o_types_slam3d
  g2o_solver_dense
  g2o_solver_csparse
  Sophus::Sophus
)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  sensor_msgs
  nav_msgs
  cv_bridge
  message_filters
  OpenCV
  Sophus
)
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)
# Node Executable
add_executable(stereo_visual_slam_node
  executables/stereo_visual_slam_node.cpp
)
target_link_libraries(stereo_visual_slam_node
  ${PROJECT_NAME}_lib
)
ament_target_dependencies(stereo_visual_slam_node
  rclcpp
)
install(TARGETS
  stereo_visual_slam_node
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()