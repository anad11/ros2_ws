cmake_minimum_required(VERSION 3.8)
project(picknplace_playmotion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(play_motion2_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

#
# ---- Build demo node ----
#

add_executable(demo
  src/demo.cpp
)

ament_target_dependencies(demo
  rclcpp
  rclcpp_action
  play_motion2_msgs
  moveit_ros_planning_interface
)

install(
  TARGETS demo
  DESTINATION lib/${PROJECT_NAME}
)

#
# ---- Testing ----
#

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
