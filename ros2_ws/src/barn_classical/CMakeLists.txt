cmake_minimum_required(VERSION 3.14)
project(barn_classical)

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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(osqp_vendor REQUIRED)
find_package(osqp REQUIRED CONFIG)
find_package(barn_core REQUIRED)
find_package(barn_msgs REQUIRED)

# Pure Track-A planning and control algorithms.
add_library(barn_classical_algos
  src/collision_checker.cpp
  src/global_planner_astar.cpp
  src/path_validator.cpp
  src/local_planner.cpp
  src/controller.cpp
  src/recovery.cpp)
target_include_directories(barn_classical_algos PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(barn_classical_algos barn_core::barn_core Eigen3::Eigen osqp::osqp)

add_executable(classical_mpc_node src/classical_mpc_node.cpp)
target_include_directories(classical_mpc_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
ament_target_dependencies(
  classical_mpc_node rclcpp sensor_msgs geometry_msgs nav_msgs visualization_msgs diagnostic_msgs std_msgs barn_msgs)
target_link_libraries(classical_mpc_node barn_classical_algos barn_core::barn_core)

# The runnable vertical-slice controller.
add_executable(goal_seeker_node src/goal_seeker_node.cpp)
target_include_directories(goal_seeker_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
ament_target_dependencies(goal_seeker_node rclcpp sensor_msgs geometry_msgs)
target_link_libraries(goal_seeker_node barn_core::barn_core)

install(TARGETS goal_seeker_node classical_mpc_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS barn_classical_algos
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_goal_seeker_math test/test_goal_seeker_math.cpp)
  target_include_directories(test_goal_seeker_math PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
  target_link_libraries(test_goal_seeker_math barn_core::barn_core)

  ament_add_gtest(test_classical_pipeline test/test_classical_pipeline.cpp)
  target_include_directories(test_classical_pipeline PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
  target_link_libraries(test_classical_pipeline barn_classical_algos barn_core::barn_core)
endif()

ament_package()
