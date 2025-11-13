#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "play_motion2_msgs/action/play_motion2.hpp"
#include "moveit/move_group_interface/move_group_interface.h"

using namespace std::chrono_literals;

class PickAndPlaceClient : public rclcpp::Node
{
public:
  using PlayMotion2 = play_motion2_msgs::action::PlayMotion2;
  using GoalHandlePlayMotion2 = rclcpp_action::ClientGoalHandle<PlayMotion2>;

  explicit PickAndPlaceClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("pick_and_place_client", options)
  {
    // Create PlayMotion2 action client
    client_ = rclcpp_action::create_client<PlayMotion2>(this, "/play_motion2");

    RCLCPP_INFO(get_logger(), "Waiting for PlayMotion2 action server...");
    if (!client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(get_logger(), "PlayMotion2 server not available after waiting.");
      rclcpp::shutdown();
      return;
    }

    // Define PlayMotion2 sequence (built-in motions)
    motions_ = {
      "move_to_start_left",
      "close_left",             // grasp
      "move_to_pose2_left",     // move to center
      "open_left"      // release
    };

    current_motion_ = 0;
  }

  //  Called externally, after the node is fully constructed
  void move_to_start_pose()
  {
    RCLCPP_INFO(get_logger(), "Moving arm_left_torso to custom start pose...");

    // Safe to create MoveGroupInterface here
    moveit::planning_interface::MoveGroupInterface move_group(shared_from_this(), "arm_left_torso");

    // // Define joint targets (custom pre-pick pose)
    // std::map<std::string, double> target_joint_values = {
    //   {"torso_lift_joint", 0.09999117011102586},
    //   {"arm_left_1_joint", 1.8822170355311911},
    //   {"arm_left_2_joint", -0.7588440798161304},
    //   {"arm_left_3_joint", -1.275038949509204},
    //   {"arm_left_4_joint", -1.6180088822247392},
    //   {"arm_left_5_joint", -1.3370852919402303},
    //   {"arm_left_6_joint", 2.2447343528133707},
    //   {"arm_left_7_joint", -0.24184312913645645}
    // };

    move_group.setStartStateToCurrentState();
    move_group.setPlanningTime(10.0);

    // move_group.setJointValueTarget(target_joint_values);
    move_group.setMaxVelocityScalingFactor(0.3);
    move_group.setMaxAccelerationScalingFactor(0.3);

  //   auto success = (move_group.move() == moveit::core::MoveItErrorCode::SUCCESS);

  //   if (success)
  //     RCLCPP_INFO(get_logger(), "Arm moved to start pose successfully.");
  //   else
  //     RCLCPP_ERROR(get_logger(), "Failed to reach start pose.");
  // }

  // Start the pick-and-place sequence
  void start_sequence()
  {
    RCLCPP_INFO(get_logger(), "Starting pick-and-place sequence...");
    send_next_motion();
  }

private:
  rclcpp_action::Client<PlayMotion2>::SharedPtr client_;
  std::vector<std::string> motions_;
  size_t current_motion_;

  void send_next_motion()
  {
    if (current_motion_ >= motions_.size()) {
      RCLCPP_INFO(get_logger(), " Pick-and-place sequence complete");
      rclcpp::shutdown();
      return;
    }

    std::string motion_name = motions_[current_motion_];
    RCLCPP_INFO(get_logger(), "Sending motion: %s", motion_name.c_str());

    auto goal_msg = PlayMotion2::Goal();
    goal_msg.motion_name = motion_name;
    goal_msg.skip_planning = false;

    auto send_goal_options = rclcpp_action::Client<PlayMotion2>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      [this, motion_name](const GoalHandlePlayMotion2::SharedPtr & handle)
      {
        if (!handle) {
          RCLCPP_ERROR(this->get_logger(), "Motion %s was rejected.", motion_name.c_str());
        } else {
          RCLCPP_INFO(this->get_logger(), " Motion %s accepted by server.", motion_name.c_str());
        }
      };

    send_goal_options.result_callback =
      [this, motion_name](const GoalHandlePlayMotion2::WrappedResult & result)
      {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success) {
          RCLCPP_INFO(this->get_logger(), "Motion %s succeeded.", motion_name.c_str());
        } else {
          RCLCPP_ERROR(this->get_logger(), "Motion %s failed: %s",
                       motion_name.c_str(), result.result->error.c_str());
        }

        // Wait 2 seconds before next motion
        RCLCPP_INFO(this->get_logger(), "Waiting 2 seconds before next motion...");
        rclcpp::sleep_for(2s);

        current_motion_++;
        this->send_next_motion();
      };

    client_->async_send_goal(goal_msg, send_goal_options);
  }
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickAndPlaceClient>();

  // ✅ Safe to move the arm now (no more bad_weak_ptr)
  node->move_to_start_pose();

  // Then start the motion sequence
  node->start_sequence();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
