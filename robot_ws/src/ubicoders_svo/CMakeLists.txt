cmake_minimum_required(VERSION 3.8)
project(ubicoders_svo)

# ==============================================
# Setup & Flags
# ==============================================
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -march=native")

# Set CMP0072 to NEW to prefer GLVND and avoid warnings when g2o finds OpenGL
cmake_policy(SET CMP0072 NEW)

set(library_name sv_lib)

# ==============================================
# Find Packages
# ==============================================

# ROS2 Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_eigen REQUIRED)

# ROS2 Messages
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Extra Packages
find_package(g2o REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)

# ==============================================
# Library Definition
# ==============================================
add_library(${library_name}
  src/svo_node.cpp
  src/frontend.cpp
  src/backend.cpp
  src/map.cpp
  src/stereo_geometry.cpp
  src/viewer.cpp
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
  cv_bridge
  tf2_eigen
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  OpenCV
)

# ==============================================
# Include Directories
# ==============================================
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${G2O_INCLUDE_DIRS}
)

# ==============================================
# Executable Definition
# ==============================================
add_executable(ubicoders_svo
  executables/svo_main.cpp
)

ament_target_dependencies(ubicoders_svo
  rclcpp
)

target_link_libraries(ubicoders_svo
  ${library_name}
)

# ==============================================
# Install Rules
# ==============================================
install(TARGETS
  ubicoders_svo
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# ==============================================
# Testing
# ==============================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
