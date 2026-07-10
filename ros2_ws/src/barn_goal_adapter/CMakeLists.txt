cmake_minimum_required(VERSION 3.14)
project(barn_goal_adapter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(barn_core REQUIRED)

add_executable(goal_adapter_node src/goal_adapter_node.cpp)
target_include_directories(goal_adapter_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
ament_target_dependencies(goal_adapter_node
  rclcpp rclcpp_action nav2_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)
target_link_libraries(goal_adapter_node barn_core::barn_core)

install(TARGETS goal_adapter_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_goal_extract test/test_goal_extract.cpp)
  target_include_directories(test_goal_extract PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
  ament_target_dependencies(test_goal_extract geometry_msgs)
  target_link_libraries(test_goal_extract barn_core::barn_core)
endif()

ament_package()
