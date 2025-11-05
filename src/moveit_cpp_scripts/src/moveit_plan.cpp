#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("moveit_incremental_plan");

    // Planning group name for Tiago
    const std::string planning_group = "arm_left_torso";

    moveit::planning_interface::MoveGroupInterface move_group(node, planning_group);

    // Always start from the current robot state
    move_group.setStartStateToCurrentState();

    // Get current pose
    geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;

    // Example: incrementally move +10cm in x, +5cm in z
    geometry_msgs::msg::Pose target_pose = current_pose;
    target_pose.position.x += 0.1;
    target_pose.position.z += 0.05;

    move_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    // Plan from current state
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
        RCLCPP_INFO(node->get_logger(), "Plan successful, executing...");
        move_group.execute(plan);
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "Planning failed!");
    }

    rclcpp::shutdown();
    return 0;
}
