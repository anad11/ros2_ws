cmake_minimum_required(VERSION 3.5)
project(pick_and_place_demo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(play_motion2_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)


add_executable(pick_and_place_client src/pick_and_place_client.cpp)
ament_target_dependencies(pick_and_place_client
  rclcpp
  rclcpp_action
  play_motion2_msgs
  moveit_ros_planning_interface
)

install(TARGETS pick_and_place_client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
