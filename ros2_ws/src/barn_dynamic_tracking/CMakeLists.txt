cmake_minimum_required(VERSION 3.14)
project(barn_dynamic_tracking)

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
find_package(visualization_msgs REQUIRED)
find_package(barn_core REQUIRED)

add_library(barn_tracking
  src/kalman.cpp
  src/clustering.cpp
  src/association.cpp
  src/ttc.cpp)
target_include_directories(barn_tracking PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(barn_tracking barn_core::barn_core)

add_executable(tracker_node src/tracker_node.cpp)
target_include_directories(tracker_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
ament_target_dependencies(tracker_node rclcpp sensor_msgs geometry_msgs visualization_msgs)
target_link_libraries(tracker_node barn_tracking barn_core::barn_core)

install(TARGETS tracker_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS barn_tracking
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_kalman test/test_kalman.cpp)
  target_include_directories(test_kalman PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
  target_link_libraries(test_kalman barn_tracking barn_core::barn_core)
endif()

ament_package()
