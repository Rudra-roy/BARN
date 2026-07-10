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
find_package(barn_core REQUIRED)

# Track-A algorithm pipeline (currently stubs; not used by the slice).
add_library(barn_classical_algos
  src/global_planner_astar.cpp
  src/path_validator.cpp
  src/local_planner.cpp
  src/controller.cpp
  src/recovery.cpp)
target_include_directories(barn_classical_algos PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(barn_classical_algos barn_core::barn_core)

# The runnable vertical-slice controller.
add_executable(goal_seeker_node src/goal_seeker_node.cpp)
target_include_directories(goal_seeker_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
ament_target_dependencies(goal_seeker_node rclcpp sensor_msgs geometry_msgs)
target_link_libraries(goal_seeker_node barn_core::barn_core)

install(TARGETS goal_seeker_node DESTINATION lib/${PROJECT_NAME})
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
endif()

ament_package()
