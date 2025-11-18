
cmake_minimum_required(VERSION 3.8)
project(basic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(basic_plan src/basic.cpp)

ament_target_dependencies(basic_plan
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

install(
  TARGETS basic_plan
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
