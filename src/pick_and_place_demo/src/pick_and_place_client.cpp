#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "play_motion2_msgs/action/play_motion2.hpp"
#include "play_motion2_msgs/srv/add_motion.hpp"
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

    // 🔹 Register custom motions automatically
    add_custom_motion("move_to_start_left",
      "Move to Start Left",
      "Custom start pose for left arm and torso",
      {"torso_lift_joint","arm_left_1_joint","arm_left_2_joint","arm_left_3_joint","arm_left_4_joint","arm_left_5_joint","arm_left_6_joint","arm_left_7_joint"},
      {0.09999, 1.882217, -0.75884, -1.27504, -1.61801, -1.33709, 2.24473, -0.24184},
      {5.0});

    add_custom_motion("move_to_pose2_left",
      "Move to Pose2 Left",
      "Second pick pose for left arm",
      {"torso_lift_joint","arm_left_1_joint","arm_left_2_joint","arm_left_3_joint","arm_left_4_joint","arm_left_5_joint","arm_left_6_joint","arm_left_7_joint"},
      {0.09309, 1.91417, 0.02856, -0.95636, -1.64450, -0.93527, 2.14104, -0.80890},
      {5.0});

    // Define PlayMotion2 sequence
    motions_ = {
      "open_left",
      "move_to_start_left",
      "close_left",
      "move_to_pose2_left",
      "open_left"
    };

    current_motion_ = 0;
  }

  void start_sequence()
  {
    RCLCPP_INFO(get_logger(), "Starting pick-and-place sequence...");
    rclcpp::sleep_for(1s);
    send_next_motion();
  }

private:
  rclcpp_action::Client<PlayMotion2>::SharedPtr client_;
  std::vector<std::string> motions_;
  size_t current_motion_;

  // 🔹 Generic function to add any motion
  void add_custom_motion(
      const std::string &key,
      const std::string &name,
      const std::string &description,
      const std::vector<std::string> &joints,
      const std::vector<double> &positions,
      const std::vector<double> &times)
  {
    auto add_motion_client =
      this->create_client<play_motion2_msgs::srv::AddMotion>("/play_motion2/add_motion");

    if (!add_motion_client->wait_for_service(10s)) {
      RCLCPP_ERROR(get_logger(), "AddMotion service not available!");
      return;
    }

    auto request = std::make_shared<play_motion2_msgs::srv::AddMotion::Request>();
    request->motion.key = key;
    request->motion.name = name;
    request->motion.usage = "user_defined";
    request->motion.description = description;
    request->motion.joints = joints;
    request->motion.positions = positions;
    request->motion.times_from_start = times;
    request->overwrite = true;

    RCLCPP_INFO(get_logger(), "Adding motion '%s'...", key.c_str());
    auto future = add_motion_client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 5s)
        == rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(get_logger(), "Motion '%s' added successfully.", key.c_str());
    }
    else
    {
      RCLCPP_WARN(get_logger(), "Timeout while adding motion '%s' (it may already exist).", key.c_str());
    }
  }

  // 🔹 Execute each motion sequentially
  void send_next_motion()
  {
    if (current_motion_ >= motions_.size()) {
      RCLCPP_INFO(get_logger(), "Pick-and-place sequence complete.");
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
          RCLCPP_INFO(this->get_logger(), "Motion %s accepted by server.", motion_name.c_str());
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

        RCLCPP_INFO(this->get_logger(), "Waiting 1 second before next motion...");
        rclcpp::sleep_for(1s);

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
  node->start_sequence();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

