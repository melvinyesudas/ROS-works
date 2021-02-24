cmake_minimum_required(VERSION 3.5)
project(example)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Compile flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wfatal-errors)

  add_compile_options(-O3) # Release
  # add_compile_options(-g)  # Debug
endif()

# Find depencanies listed in package.xml
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

add_library(example_config SHARED example_config.cc)
ament_target_dependencies(example_config "rclcpp" "rclcpp_components")
rclcpp_components_register_nodes(example_config "ExampleConfig")

add_library(example_pub SHARED example_pub.cc)
ament_target_dependencies(example_pub "rclcpp" "rclcpp_components" "example_msgs")
rclcpp_components_register_nodes(example_pub "ExamplePub")

add_library(example_sub SHARED example_sub.cc)
ament_target_dependencies(example_sub "rclcpp" "rclcpp_components" "example_msgs")
rclcpp_components_register_nodes(example_sub "ExampleSub")

add_library(example_status SHARED example_status.cc)
ament_target_dependencies(example_status "rclcpp" "rclcpp_components" "diagnostic_msgs")
rclcpp_components_register_nodes(example_status "ExampleStatus")

add_library(example_service SHARED example_service.cc)
ament_target_dependencies(example_service "rclcpp" "rclcpp_components" "example_msgs")
rclcpp_components_register_nodes(example_service "ExampleService")

add_library(example_client SHARED example_client.cc)
ament_target_dependencies(example_client "rclcpp" "rclcpp_components" "example_msgs")
rclcpp_components_register_nodes(example_client "ExampleClient")

add_library(example_time SHARED example_time.cc)
ament_target_dependencies(example_time "rclcpp" "rclcpp_components")
rclcpp_components_register_nodes(example_time "ExampleTime")

add_library(example_log SHARED example_log.cc)
ament_target_dependencies(example_log "rclcpp" "rclcpp_components")
rclcpp_components_register_nodes(example_log "ExampleLog")

add_library(example_tf SHARED example_tf.cc)
ament_target_dependencies(example_tf REQUIRED "rclcpp" "rclcpp_components" "tf2_ros" "tf2_geometry_msgs" "geometry_msgs")
rclcpp_components_register_nodes(example_tf "ExampleTF")

add_library(example_tf_filter SHARED example_tf_filter.cc)
ament_target_dependencies(example_tf_filter REQUIRED "rclcpp" "rclcpp_components" "tf2_ros" "tf2_geometry_msgs" "geometry_msgs")
rclcpp_components_register_nodes(example_tf_filter "ExampleTF_filter")

# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

install(TARGETS
  example_config
  example_pub
  example_sub
  example_status
  example_service
  example_client
  example_time
  example_log
  example_tf
  example_tf_filter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_lint_auto_find_test_dependencies()

  # Add a gtest
  ament_add_gtest(gtest_${PROJECT_NAME}_test test/test.cc TIMEOUT 10)
endif()



ament_package()
