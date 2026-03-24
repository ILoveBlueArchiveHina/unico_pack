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

add_executable(mission_dispatcher src/mission_dispatcher.cpp)
ament_target_dependencies(mission_dispatcher
  rclcpp 
  rclcpp_action
  nav2_msgs
  geometry_msgs 
  std_msgs 
  tf2 
  tf2_ros
  campus_delivery_msgs
)

install(TARGETS
  multi_marker_landing_node
  mission_dispatcher
  DESTINATION lib/${PROJECT_NAME}
)

install (
  DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
  src/cmd_vel_bridge.py
  src/multi_marker_landing_v3.py
  src/mission_dispatcher_with_tracker_v2.py
  src/manager.py
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
