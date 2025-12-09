cmake_minimum_required(VERSION 3.8)
project(euroc_image_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(euroc_image_publisher_node src/euroc_image_publisher_node.cpp)
target_include_directories(euroc_image_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(euroc_image_publisher_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  OpenCV
)

target_link_libraries(euroc_image_publisher_node
  ${yaml-cpp_LIBRARIES}
  yaml-cpp
)

install(TARGETS euroc_image_publisher_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
