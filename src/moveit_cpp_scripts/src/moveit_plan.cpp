// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <geometry_msgs/msg/pose.hpp>

// int main(int argc, char** argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = rclcpp::Node::make_shared("go_to_start");

//     rclcpp::executors::SingleThreadedExecutor executor;
//     executor.add_node(node);
//     std::thread([&executor]() { executor.spin(); }).detach();
//     rclcpp::sleep_for(std::chrono::seconds(2));


//     // Planning group name for Tiago
//     static const std::string PLANNING_GROUP = "arm_left_torso";
//     moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
//     move_group.setPlanningTime(10.0);

//     moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
//     const moveit::core::JointModelGroup* joint_model_group =
//         move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    

//     std::vector<double> joint_group_positions;
//     current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
//     joint_group_positions[0] = -1.0;

    
//     // std :: vector <double> start_pose = {
//     //     1.5781,   // arm_left_1_joint
//     //    -0.5320,   // arm_left_2_joint
//     //    -2.3198,   // arm_left_3_joint
//     //    -1.4679,   // arm_left_4_joint
//     //    -1.1135,   // arm_left_5_joint
//     //     1.7486,   // arm_left_6_joint
//     //     0.6406    // arm_left_7_joint
//     // };


//     //move_group.setStartStateToCurrentState();
//     move_group.setJointValueTarget(joint_group_positions);
//     moveit::planning_interface::MoveGroupInterface::Plan plan;
//     bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
//     if (success)
//     {
//         RCLCPP_INFO(node->get_logger(), "Plan to start pose successful, executing...");
//         move_group.execute(plan);
//     }
//     else
//     {
//         RCLCPP_WARN(node->get_logger(), "Planning to start pose failed!");
//         return 1;
//     }


//     rclcpp::shutdown();
//     return 0;
// }
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <thread>
#include <chrono>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // Create NodeOptions and set use_sim_time parameter
    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {rclcpp::Parameter("use_sim_time", true)}
    );

    auto node = rclcpp::Node::make_shared("go_to_start", options);

    // Executor to spin the node
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    rclcpp::sleep_for(std::chrono::seconds(2));

    static const std::string PLANNING_GROUP = "arm_left_torso";
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
    move_group.setPlanningTime(10.0);

    auto wait_for_state = [&move_group]() {
    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
        auto state = move_group.getCurrentState();
        if (state && state->satisfiesBounds()) {
            break;
        }
        RCLCPP_WARN(move_group.getNode()->get_logger(), "Waiting for valid current robot state...");
        rate.sleep();
        }
    };
    wait_for_state();


    auto current_state = move_group.getCurrentState(10);
    const moveit::core::JointModelGroup* joint_model_group =
        current_state->getJointModelGroup(PLANNING_GROUP);

    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    joint_group_positions[0] = -1.0;

    move_group.setJointValueTarget(joint_group_positions);

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
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
