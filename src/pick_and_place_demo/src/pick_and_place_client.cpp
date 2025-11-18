#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "play_motion2_msgs/action/play_motion2.hpp"
#include "play_motion2_msgs/srv/add_motion.hpp"

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include <moveit/robot_state/robot_state.h>

using namespace std::chrono_literals;

// Struct for user-provided offset
struct Offset {
  double dx = 0.0;
  double dy = 0.0;
  double dz = 0.0;
};

class PickAndPlaceClient : public rclcpp::Node
{
public:
  using PlayMotion2 = play_motion2_msgs::action::PlayMotion2;
  using GoalHandlePlayMotion2 = rclcpp_action::ClientGoalHandle<PlayMotion2>;

  explicit PickAndPlaceClient(const Offset &offset)
    : Node(
        "pick_and_place_client",
        rclcpp::NodeOptions().arguments({
            "--ros-args",
            "-r", "/joint_states:=/joint_states_best_effort"
        })
      )
    , offset_(offset)


  {
    // PlayMotion2 action client
    if (!this->has_parameter("use_sim_time"))
        this->declare_parameter<bool>("use_sim_time", true);

    this->set_parameter(rclcpp::Parameter("use_sim_time", true));
    RCLCPP_INFO(get_logger(), "Using simulation time.");
    pm_client_ = rclcpp_action::create_client<PlayMotion2>(this, "/play_motion2");
    pm_client_->wait_for_action_server();

    // --- Register custom motions ---
    add_motion(
      "move_to_start_left",
      {
        "torso_lift_joint","arm_left_1_joint","arm_left_2_joint","arm_left_3_joint",
        "arm_left_4_joint","arm_left_5_joint","arm_left_6_joint","arm_left_7_joint"
      },
      {0.09999, 1.882217, -0.75884, -1.27504, -1.61801, -1.33709, 2.24473, -0.24184}
    );

    motions_ = { "open_left", "move_to_start_left", "close_left", "open_left" };
  }

  void initialize_moveit()
  {
    if (moveit_initialized_) return;

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "arm_left");

    move_group_->setPlanningTime(5.0);
    move_group_->startStateMonitor();

    moveit_initialized_ = true;

    RCLCPP_INFO(get_logger(), "MoveIt MoveGroup initialized.");
  }

  void start()
  {
    initialize_moveit();
    current_motion_ = 0;
    execute_next();
  }

private:

  // ----------------------------------------------------
  // Add PlayMotion2 motion
  // ----------------------------------------------------
  void add_motion(const std::string &key,
                  const std::vector<std::string> &joints,
                  const std::vector<double> &positions)
  {
    auto client =
      this->create_client<play_motion2_msgs::srv::AddMotion>("/play_motion2/add_motion");

    if (!client->wait_for_service(5s)) {
      RCLCPP_ERROR(get_logger(), "AddMotion service unavailable!");
      return;
    }

    auto req = std::make_shared<play_motion2_msgs::srv::AddMotion::Request>();
    req->motion.key = key;
    req->motion.name = key;
    req->motion.joints = joints;
    req->motion.positions = positions;
    req->motion.times_from_start = {5.0};
    req->overwrite = true;

    auto future = client->async_send_request(req);
    rclcpp::spin_until_future_complete(get_node_base_interface(), future, 3s);

    RCLCPP_INFO(get_logger(), "Registered motion '%s'", key.c_str());
  }

  // ----------------------------------------------------
  // Run a PlayMotion2 motion, then call callback()
  // ----------------------------------------------------
  void run_motion(const std::string &name, std::function<void()> callback)
  {
    auto goal = PlayMotion2::Goal();
    goal.motion_name = name;
    goal.skip_planning = false;

    auto options = rclcpp_action::Client<PlayMotion2>::SendGoalOptions();

    options.goal_response_callback =
      [this, name](GoalHandlePlayMotion2::SharedPtr handle)
      {
        if (!handle)
          RCLCPP_ERROR(this->get_logger(), "Motion %s rejected", name.c_str());
        else
          RCLCPP_INFO(this->get_logger(), "Motion %s accepted", name.c_str());
      };

    options.result_callback =
      [this, name, callback](const GoalHandlePlayMotion2::WrappedResult &result)
      {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
            result.result->success)
        {
          RCLCPP_INFO(this->get_logger(), "Motion %s succeeded", name.c_str());
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Motion %s failed", name.c_str());
        }

        rclcpp::sleep_for(1s);
        callback();
      };

    pm_client_->async_send_goal(goal, options);
  }

  // ----------------------------------------------------
  // MoveIt offset step
  // ----------------------------------------------------
  void do_moveit_offset()
  {
    RCLCPP_INFO(get_logger(), "Planning MoveIt offset...");

    auto state = move_group_->getCurrentState(5.0);
    if (!state) {
      RCLCPP_ERROR(get_logger(), "MoveIt failed to get current state.");
      return execute_next();
    }

    Eigen::Isometry3d pose =
      state->getGlobalLinkTransform("arm_left_tool_link");

    pose.translation().x() += offset_.dx;
    pose.translation().y() += (-0.15 + offset_.dy);  // inward
    pose.translation().z() += offset_.dz;

    move_group_->setPoseTarget(pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "MoveIt planning failed.");
      return execute_next();
    }

    move_group_->execute(plan);
    rclcpp::sleep_for(1s);
    execute_next();
  }

  // ----------------------------------------------------
  // Motion sequence
  // ----------------------------------------------------
  void execute_next()
  {
    if (current_motion_ >= motions_.size()) {
      RCLCPP_INFO(get_logger(), "Sequence finished.");
      rclcpp::shutdown();
      return;
    }

    std::string m = motions_[current_motion_++];
    if (m == "move_to_start_left") {
      run_motion(m, [this]() { do_moveit_offset(); });
    }
    else {
      run_motion(m, [this]() { execute_next(); });
    }
  }

  // MEMBERS
  Offset offset_;
  rclcpp_action::Client<PlayMotion2>::SharedPtr pm_client_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  bool moveit_initialized_ = false;

  std::vector<std::string> motions_;
  size_t current_motion_ = 0;
};

// ------------------------------------------------------------
// MAIN
// ------------------------------------------------------------
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  Offset off;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a[0] == 'x') off.dx = std::stod(a.substr(1));
    if (a[0] == 'y') off.dy = std::stod(a.substr(1));
    if (a[0] == 'z') off.dz = std::stod(a.substr(1));
  }

  auto node = std::make_shared<PickAndPlaceClient>(off);
  rclcpp::spin_some(node);
  node->initialize_moveit();   
  node->start();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
