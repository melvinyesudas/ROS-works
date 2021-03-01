cmake_minimum_required(VERSION 3.5)
project(visualizer)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g3 -Og)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trackedobj_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/)
include_directories(${EIGEN3_INCLUDE_DIR})

add_library(visualizer SHARED VisualizationMarker.cpp)

ament_target_dependencies(visualizer "nav_msgs" "rclcpp" "rclcpp_components" "diagnostic_msgs"
  "rcutils"  "geometry_msgs" "lidar_detection_msgs" "trackedobj_msgs" "Eigen3" "visualization_msgs" "tf2" "tf2_ros" "std_msgs")

rclcpp_components_register_nodes(visualizer "VisualizationMarker")

# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

install(TARGETS
  visualizer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

  # if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_lint_auto_find_test_dependencies()

  # #Add a gtest
  # ament_add_gtest(gtest_${PROJECT_NAME}_test test/test.cc TIMEOUT 10)
  # endif()


ament_package()
