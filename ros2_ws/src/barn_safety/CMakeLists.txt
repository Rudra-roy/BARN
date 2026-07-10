cmake_minimum_required(VERSION 3.14)
project(barn_safety)

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

# Pure limiter logic (unit-tested without ROS).
add_library(barn_limiter src/limiter.cpp)
target_include_directories(barn_limiter PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(barn_limiter barn_core::barn_core)

add_executable(safety_node src/safety_node.cpp)
target_include_directories(safety_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
ament_target_dependencies(safety_node rclcpp sensor_msgs geometry_msgs)
target_link_libraries(safety_node barn_limiter barn_core::barn_core)

install(TARGETS safety_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS barn_limiter
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_limiter test/test_limiter.cpp)
  target_include_directories(test_limiter PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
  target_link_libraries(test_limiter barn_limiter barn_core::barn_core)
endif()

ament_package()
