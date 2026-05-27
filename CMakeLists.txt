cmake_minimum_required(VERSION 3.8)
project(unico_pack)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(aruco_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(campus_delivery_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(multi_marker_landing_node src/multi_marker_landing.cpp)
ament_target_dependencies(multi_marker_landing_node 
  rclcpp 
  geometry_msgs 
  sensor_msgs 
  nav_msgs 
  mavros_msgs 
  aruco_msgs 
  std_msgs 
  tf2 
  tf2_ros
)

add_executable(nav_velocity_tracker src/nav_velocity_tracker.cpp)
ament_target_dependencies(nav_velocity_tracker
  rclcpp 
  nav2_msgs
  geometry_msgs 
  std_msgs 
  tf2 
  tf2_ros
)

add_executable(velocity_controller src/velocity_controller.cpp)
ament_target_dependencies(velocity_controller
  rclcpp 
  nav2_msgs
  geometry_msgs 
  std_msgs 
  tf2 
  tf2_ros
)

add_executable(precision_landing_lifecycle src/precision_landing_lifecycle.cpp)
ament_target_dependencies(precision_landing_lifecycle
  rclcpp 
  rclcpp_lifecycle 
)

add_executable(fast_lio_lifecycle_wrapper src/fast_lio_lifecycle_wrapper.cpp)
ament_target_dependencies(fast_lio_lifecycle_wrapper
  rclcpp 
  rclcpp_lifecycle 
)

add_executable(rosbag_lifecycle src/rosbag_lifecycle.cpp)
ament_target_dependencies(rosbag_lifecycle 
  rclcpp 
  rclcpp_lifecycle
)

add_executable(mission_dispatcher_v3 src/mission_dispatcher_v3.cpp)
ament_target_dependencies(mission_dispatcher_v3
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  nav2_msgs
  campus_delivery_msgs
)

add_executable(cmd_vel_bridge src/cmd_vel_bridge.cpp)
ament_target_dependencies(cmd_vel_bridge
  rclcpp 
  geometry_msgs 
  mavros_msgs 
  std_msgs
)

install(TARGETS
  multi_marker_landing_node
  nav_velocity_tracker
  velocity_controller
  mission_dispatcher_v3
  cmd_vel_bridge
  precision_landing_lifecycle
  fast_lio_lifecycle_wrapper
  rosbag_lifecycle
  DESTINATION lib/${PROJECT_NAME}
)

install (
  DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

execute_process(
  COMMAND chmod +x
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mission_dispatcher_v2.py
    ${CMAKE_CURRENT_SOURCE_DIR}/src/manager.py
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.py
)

install(
  PROGRAMS
  src/mission_dispatcher_v2.py
  src/mission_dispatcher_v4.py
  src/manager.py
  src/main.py
  DESTINATION lib/${PROJECT_NAME}
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
