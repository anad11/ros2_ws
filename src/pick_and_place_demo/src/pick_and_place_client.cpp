#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "play_motion2_msgs/action/play_motion2.hpp"

using namespace std::chrono_literals;

class PickAndPlaceClient : public rclcpp::Node
{
public:
  using PlayMotion2 = play_motion2_msgs::action::PlayMotion2;
  using GoalHandlePlayMotion2 = rclcpp_action::ClientGoalHandle<PlayMotion2>;

  explicit PickAndPlaceClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("pick_and_place_client", options)
  {
    client_ = rclcpp_action::create_client<PlayMotion2>(this, "/play_motion2");

    RCLCPP_INFO(get_logger(), "Waiting for PlayMotion2 action server...");
    if (!client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(get_logger(), "PlayMotion2 server not available after waiting.");
      rclcpp::shutdown();
      return;
    }

    // Pick-and-place sequence using built-in TIAGo motions
    motions_ = {
      "home",         // start pose
      "open_left",    // open gripper
      "offer_left",   // move to grasp position (top-right)
      "close_left",   // close gripper (simulate pick)
      "home_left",    // move to center
      "open_left"     // open gripper (release)
    };

    current_motion_ = 0;
    send_next_motion();
  }

private:
  rclcpp_action::Client<PlayMotion2>::SharedPtr client_;
  std::vector<std::string> motions_;
  size_t current_motion_;

  void send_next_motion()
  {
    if (current_motion_ >= motions_.size()) {
      RCLCPP_INFO(get_logger(), "✅ Pick-and-place sequence complete!");
      rclcpp::shutdown();
      return;
    }

    std::string motion_name = motions_[current_motion_];
    RCLCPP_INFO(get_logger(), "➡️ Sending motion: %s", motion_name.c_str());

    auto goal_msg = PlayMotion2::Goal();
    goal_msg.motion_name = motion_name;
    goal_msg.skip_planning = false;

    auto send_goal_options = rclcpp_action::Client<PlayMotion2>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      [this, motion_name](const GoalHandlePlayMotion2::SharedPtr & handle)
      {
        if (!handle) {
          RCLCPP_ERROR(this->get_logger(), "❌ Motion %s was rejected.", motion_name.c_str());
        } else {
          RCLCPP_INFO(this->get_logger(), "✅ Motion %s accepted by server.", motion_name.c_str());
        }
      };

    send_goal_options.result_callback =
      [this, motion_name](const GoalHandlePlayMotion2::WrappedResult & result)
      {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result->success) {
          RCLCPP_INFO(this->get_logger(), "🏁 Motion %s succeeded.", motion_name.c_str());
        } else {
          RCLCPP_ERROR(this->get_logger(), "⚠️ Motion %s failed: %s",
                       motion_name.c_str(), result.result->error.c_str());
        }

        // Add a small delay between motions
        RCLCPP_INFO(this->get_logger(), "⏸️ Waiting 2 seconds before next motion...");
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
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

