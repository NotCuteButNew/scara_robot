cmake_minimum_required(VERSION 3.8)
project(scara_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(controller_manager REQUIRED)

add_library(
  scara_control
  SHARED
  src/scara_hardware_interafce.cpp
)
target_include_directories(
  scara_control
  PUBLIC
  include
)
ament_target_dependencies(
  scara_control
  hardware_interface
  rclcpp
  rclcpp_lifecycle
)
# prevent pluginlib from using boost
target_compile_definitions(scara_control PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface scara_control.xml)

install(
  TARGETS
  scara_control
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_scara_hardware_interafce test/test_scara_hardware_interafce.cpp)
  target_include_directories(test_scara_hardware_interafce PRIVATE include)
  ament_target_dependencies(
    test_scara_hardware_interafce
    hardware_interface
    pluginlib
    ros2_control_test_assets
  )
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  scara_control
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/launch
)