# Copyright 2026 barn-2027-prep contributors. MIT License.
#
# Standalone build for the barn_dynamic_obstacle Gazebo Sim system plugin.
# Kept out of the ament/colcon workspace so it builds independently.
#
# Requires the gz vendor packages from ROS Jazzy on the CMake prefix path:
#   source /opt/ros/jazzy/setup.bash
cmake_minimum_required(VERSION 3.16)
project(barn_dynamic_obstacle CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Proven gz-sim8 (Harmonic) discovery recipe for this box.
find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)

add_library(barn_dynamic_obstacle SHARED src/DynamicObstacle.cc)

target_link_libraries(barn_dynamic_obstacle
  gz-sim8::gz-sim8
  gz-plugin2::register)
