cmake_minimum_required(VERSION 3.14)
project(barn_mapping)

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

add_library(barn_mapping_core src/ray_integrator.cpp)
target_include_directories(barn_mapping_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(barn_mapping_core barn_core::barn_core)

add_executable(mapping_node src/mapping_node.cpp)
target_include_directories(mapping_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
ament_target_dependencies(mapping_node rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros)
target_link_libraries(mapping_node barn_mapping_core barn_core::barn_core)

install(TARGETS mapping_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS barn_mapping_core
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_logodds test/test_logodds.cpp)
  target_include_directories(test_logodds PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
  target_link_libraries(test_logodds barn_mapping_core barn_core::barn_core)
endif()

ament_package()
