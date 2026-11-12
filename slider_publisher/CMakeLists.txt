# SPDX-FileCopyrightText: 2025, FANUC America Corporation
# SPDX-FileCopyrightText: 2025, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.8)
project(slider_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(qt_gui REQUIRED)
find_package(qt_gui_cpp REQUIRED)
find_package(Qt5 COMPONENTS Widgets Core REQUIRED)

# Add executable
add_executable(slider_gui_node src/slider_gui_node.cpp)
target_link_libraries(slider_gui_node ${rclcpp_TARGETS} ${std_msgs_TARGETS} ${sensor_msgs_TARGETS})
target_link_libraries(slider_gui_node Qt5::Widgets Qt5::Core)

# Install executable
install(TARGETS
  slider_gui_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
