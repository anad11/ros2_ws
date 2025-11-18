cmake_minimum_required(VERSION 3.5)
project(pick_and_place_demo)

# ---------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(play_motion2_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)


add_executable(pick_and_place_client
  src/pick_and_place_client.cpp
)

ament_target_dependencies(pick_and_place_client
  rclcpp
  rclcpp_action
  play_motion2_msgs
  moveit_ros_planning_interface
)


add_executable(joint_state_qos_bridge
  src/joint_state_qos_bridge.cpp
)

ament_target_dependencies(joint_state_qos_bridge
  rclcpp
  sensor_msgs
)


install(
  TARGETS
    pick_and_place_client
    joint_state_qos_bridge
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
