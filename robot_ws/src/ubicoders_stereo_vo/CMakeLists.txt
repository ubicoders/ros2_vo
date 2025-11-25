cmake_minimum_required(VERSION 3.8)
project(ubicoders_stereo_vo)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -march=native")


set(library_name sv_lib)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(cv_bridge REQUIRED)
cmake_policy(SET CMP0072 NEW) 

find_package(g2o REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)


add_library(${library_name}
  src/ubicoders_svo_publishers.cpp
  # src/frontend.cpp
  # src/backend.cpp
  # src/map_point.cpp
  # src/map.cpp
  # src/frame.cpp
  # src/feature.cpp
  # src/config.cpp
  # src/pinhole_camera.cpp
  # src/viewer.cpp
)

target_link_libraries(${library_name}
  g2o_core
  g2o_stuff
  g2o_types_sba
  g2o_types_slam3d
  g2o_solver_dense
  g2o_solver_csparse
  Sophus::Sophus
)

ament_target_dependencies(${library_name}
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  cv_bridge
  OpenCV
  tf2_eigen
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${G2O_INCLUDE_DIRS}
)


add_executable(ubicoders_svo_node
  src/ubicoders_svo_node.cpp
)

ament_target_dependencies(ubicoders_svo_node
  rclcpp
)

target_link_libraries(ubicoders_svo_node
  ${library_name}
)

install(TARGETS
  ubicoders_svo_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
