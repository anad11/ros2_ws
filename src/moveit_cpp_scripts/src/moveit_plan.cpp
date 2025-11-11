#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <chrono>  // For std::chrono::seconds

int main(int argc, char** argv)
{
    // 1. Initialize ROS 2
    rclcpp::init(argc, argv);

    // 2. Create node options and set 'use_sim_time' parameter
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("use_sim_time", true)
    });

    // Create the node
    auto node = rclcpp::Node::make_shared("moveit_incremental_plan", options);

    // Optional parameters for planning
    node->declare_parameter<double>("wait_for_state_timeout", 5.0);
    node->declare_parameter<double>("planning_time", 10.0);

    RCLCPP_INFO(node->get_logger(), "Starting MoveGroupInterface setup.");

    // 3. Setup MoveGroupInterface
    const std::string planning_group = "arm_left_torso";
    moveit::planning_interface::MoveGroupInterface move_group(node, planning_group);

    // Retrieve and apply planning time
    double planning_time;
    node->get_parameter("planning_time", planning_time);
    move_group.setPlanningTime(planning_time);

    // 4. Set the start state
    move_group.setStartStateToCurrentState();

    // 5. Define the target pose (incremental move)
    geometry_msgs::msg::Pose current_pose;
    try {
        current_pose = move_group.getCurrentPose().pose;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Could not get current pose: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    // Offset the pose slightly
    auto target_pose = current_pose;
    target_pose.position.x += 0.1;  // Move 10 cm forward
    target_pose.position.z += 0.05; // Move 5 cm upward

    move_group.setPoseTarget(target_pose);

    RCLCPP_INFO(node->get_logger(), "Attempting to plan to target pose:");
    RCLCPP_INFO(node->get_logger(), "  x: %.3f, y: %.3f, z: %.3f",
                target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z);

    // 6. Plan the motion
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    // 7. Execute or report failure
    if (success) {
        RCLCPP_INFO(node->get_logger(), "Plan successful, executing...");
        moveit::core::MoveItErrorCode result = move_group.execute(plan);
        if (result == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(node->get_logger(), "Execution finished successfully.");
        } else {
            RCLCPP_WARN(node->get_logger(), "Execution failed with code: %d",
                        static_cast<int>(result.val));
        }
    } else {
        RCLCPP_WARN(node->get_logger(),
                    "Planning failed! Check if robot state is synchronized and target is reachable.");
    }

    // 8. Keep alive briefly, then shut down
    rclcpp::sleep_for(std::chrono::seconds(5));
    RCLCPP_INFO(node->get_logger(), "Shutting down.");
    rclcpp::shutdown();
    return 0;
}
