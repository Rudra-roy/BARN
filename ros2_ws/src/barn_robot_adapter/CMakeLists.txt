cmake_minimum_required(VERSION 3.14)
project(barn_robot_adapter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(barn_core REQUIRED)

# Reusable, ROS-boundary conversion library (unit-tested independently).
add_library(barn_robot_conversions src/conversions.cpp)
target_include_directories(barn_robot_conversions PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(barn_robot_conversions sensor_msgs nav_msgs geometry_msgs)
target_link_libraries(barn_robot_conversions barn_core::barn_core)

add_executable(robot_adapter_node src/robot_adapter_node.cpp)
target_include_directories(robot_adapter_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
ament_target_dependencies(robot_adapter_node
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros)
target_link_libraries(robot_adapter_node barn_robot_conversions barn_core::barn_core)

install(TARGETS robot_adapter_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS barn_robot_conversions
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_conversions test/test_conversions.cpp)
  target_include_directories(test_conversions PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
  ament_target_dependencies(test_conversions sensor_msgs nav_msgs geometry_msgs)
  target_link_libraries(test_conversions barn_robot_conversions barn_core::barn_core)
endif()

ament_package()
