cmake_minimum_required(VERSION 3.5)
project(limo_control)

# C++ settings:
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -g)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)

# Declare the limo_controller executable
add_executable(limo_controller
  limo_controller/limo_controller_node.cpp
)

# Specify include directories
target_include_directories(limo_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/limo_controller>
  $<INSTALL_INTERFACE:include>
)

# Specify libraries to link
ament_target_dependencies(limo_controller
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
)

# Install the executable
install(TARGETS limo_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(FILES
  limo_controller/limo_controller_node.hpp
  DESTINATION include/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()