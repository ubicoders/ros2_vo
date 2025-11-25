cmake_minimum_required(VERSION 3.8)
project(kitti_image_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(kitti_image_publisher_node
  src/kitti_image_publisher_node.cpp
)
ament_target_dependencies(kitti_image_publisher_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

target_include_directories(kitti_image_publisher_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  kitti_image_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
