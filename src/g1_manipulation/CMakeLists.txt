cmake_minimum_required(VERSION 3.16)
project(g1_manipulation)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(g1_interfaces REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(move_to_pose src/move_to_pose.cpp)
target_compile_features(move_to_pose PUBLIC cxx_std_17)
ament_target_dependencies(
  move_to_pose
  geometry_msgs
  moveit_ros_planning_interface
  rclcpp
  shape_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS move_to_pose DESTINATION lib/${PROJECT_NAME})
add_executable(pick_server src/pick_server.cpp)
target_compile_features(pick_server PUBLIC cxx_std_17)
ament_target_dependencies(pick_server
  g1_interfaces rclcpp rclcpp_action moveit_ros_planning_interface geometry_msgs
)
install(TARGETS pick_server DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
