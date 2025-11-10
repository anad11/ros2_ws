#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("go_to_start");

    // Planning group name for Tiago
    static const std::string PLANNING_GROUP = "arm_left_torso";

    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);


    std :: vector <double> start_pose = {
        1.5781,   // arm_left_1_joint
       -0.5320,   // arm_left_2_joint
       -2.3198,   // arm_left_3_joint
       -1.4679,   // arm_left_4_joint
       -1.1135,   // arm_left_5_joint
        1.7486,   // arm_left_6_joint
        0.6406    // arm_left_7_joint
    };


    move_group.setStartStateToCurrentState();
    move_group.setJointValueTarget(start_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success)
    {
        RCLCPP_INFO(node->get_logger(), "Plan to start pose successful, executing...");
        move_group.execute(plan);
    }
    else
    {
        RCLCPP_WARN(node->get_logger(), "Planning to start pose failed!");
        return 1;
    }


    rclcpp::shutdown();
    return 0;
}
