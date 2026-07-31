cmake_minimum_required(VERSION 3.8)
project(rm_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rm_ros_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

# Build the hardware interface libraries
add_library(rm_hardware SHARED
  src/rm_ros2_control_hardware.cpp
  src/rm_sixforce_hardware.cpp
)

target_include_directories(rm_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(rm_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rm_ros_interfaces
  sensor_msgs
  realtime_tools
  geometry_msgs
)

# Export the pluginlib plugins
pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

# Install the library
install(TARGETS rm_hardware
  EXPORT export_rm_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install plugin description file
install(FILES hardware_interface_plugin.xml
  DESTINATION share/${PROJECT_NAME}/
)

# Install config and launch files
install(DIRECTORY config launch urdf rviz  meshes
  DESTINATION share/${PROJECT_NAME}/
)

# Export the package
ament_export_targets(export_rm_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rm_ros_interfaces
  sensor_msgs
  realtime_tools
  geometry_msgs
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
endif()

ament_package()
