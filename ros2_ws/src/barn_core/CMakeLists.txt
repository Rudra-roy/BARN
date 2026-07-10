cmake_minimum_required(VERSION 3.14)
project(barn_core)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_library(barn_core
  src/geometry.cpp
  src/scan.cpp
  src/occupancy.cpp)
target_include_directories(barn_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_compile_features(barn_core PUBLIC cxx_std_17)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS barn_core
  EXPORT export_barn_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(export_barn_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_geometry test/test_geometry.cpp)
  target_link_libraries(test_geometry barn_core)

  ament_add_gtest(test_scan test/test_scan.cpp)
  target_link_libraries(test_scan barn_core)
endif()

ament_package()
