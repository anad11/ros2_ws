#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_msgs/msg/string.hpp"

#include "play_motion2_msgs/action/play_motion2.hpp"
#include "play_motion2_msgs/srv/add_motion.hpp"

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include <moveit/robot_state/robot_state.h>

using namespace std::chrono_literals;

// ======================================================================
// OFFSET STRUCT
// ======================================================================
struct Offset {
  double dx = 0, dy = 0, dz = 0;
};

// ======================================================================
// MAIN CLASS
// ======================================================================
class PickAndPlaceClient : public rclcpp::Node
{
public:
  using PlayMotion2 = play_motion2_msgs::action::PlayMotion2;
  using GoalHandlePlayMotion2 = rclcpp_action::ClientGoalHandle<PlayMotion2>;

  explicit PickAndPlaceClient(const Offset & off)
  : rclcpp::Node("pick_and_place_client"), offset_(off)
  {
    // Subscribe to robot_description (latched)
    urdf_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/robot_description",
      rclcpp::QoS(10).transient_local().reliable(),
      std::bind(&PickAndPlaceClient::onURDF, this, std::placeholders::_1));

    // PlayMotion2 client
    pm_client_ = rclcpp_action::create_client<PlayMotion2>(this, "/play_motion2");

    // Start timeout timer (fires once after 3 sec)
    timer_ = this->create_wall_timer(
      3s, std::bind(&PickAndPlaceClient::timeoutCheck, this));
  }

private:

  // ======================================================================
  // CALLBACK: robot_description received
  // ======================================================================
  void onURDF(const std_msgs::msg::String::SharedPtr msg)
  {
    if (robot_description_received_) return;

    robot_description_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Received robot_description.");

    timer_->cancel();   // stop timeout timer
    initialize_moveit(); // safe to continue
  }

  // ======================================================================
  // TIMEOUT CHECK — runs once
  // ======================================================================
  void timeoutCheck()
  {
    if (!robot_description_received_) {
      RCLCPP_ERROR(this->get_logger(),
        "Timed out waiting for /robot_description (3 sec). Exiting.");
      rclcpp::shutdown();
    }
  }

  // ======================================================================
  // INITIALIZE MOVEIT (safe, no loops)
  // ======================================================================
  void initialize_moveit()
  {
    if (!pm_client_->wait_for_action_server(2s)) {
      RCLCPP_ERROR(get_logger(), "PlayMotion2 server not available. Exiting.");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(get_logger(), "Initializing MoveGroup interface...");

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "arm_left");

    move_group_->setPlanningTime(5.0);
    move_group_->setNumPlanningAttempts(3);

    setup_start_motion();
    compute_pose2_and_register();
    run_sequence();
  }

  // ======================================================================
  // REGISTER FIXED START MOTION
  // ======================================================================
  void setup_start_motion()
  {
    start_joints_ = {
      0.10,   // torso
      1.88, -0.75, -1.27, -1.61, -1.33, 2.24, -0.24
    };

    add_motion("move_to_start_left", start_joints_);
  }

  // ======================================================================
  // GENERIC ADD-MOTION CALL
  // ======================================================================
  void add_motion(const std::string & key, const std::vector<double> & joints)
  {
    auto client = this->create_client<play_motion2_msgs::srv::AddMotion>(
      "/play_motion2/add_motion");

    if (!client->wait_for_service(2s)) {
      RCLCPP_ERROR(get_logger(), "Cannot call AddMotion — service down.");
      return;
    }

    auto req = std::make_shared<play_motion2_msgs::srv::AddMotion::Request>();

    req->motion.key = key;
    req->motion.joints = {
      "torso_lift_joint",
      "arm_left_1_joint","arm_left_2_joint","arm_left_3_joint",
      "arm_left_4_joint","arm_left_5_joint","arm_left_6_joint","arm_left_7_joint"
    };
    req->motion.positions = joints;
    req->motion.times_from_start = {5.0};
    req->motion.usage = "user_defined";
    req->overwrite = true;

    client->async_send_request(req);
  }

  // ======================================================================
  // COMPUTE POSE2
  // ======================================================================
  void compute_pose2_and_register()
  {
    auto model = move_group_->getRobotModel();
    auto * jmg = model->getJointModelGroup("arm_left");

    std::vector<double> arm_joints(start_joints_.begin() + 1, start_joints_.end());
    moveit::core::RobotState s(model);
    s.setJointGroupPositions(jmg, arm_joints);

    auto pose = s.getGlobalLinkTransform("arm_left_tool_link");

    pose.translation().x() += 0.15 + offset_.dx;
    pose.translation().y() += offset_.dy;
    pose.translation().z() += offset_.dz;

    moveit::core::RobotState ik(model);
    if (!ik.setFromIK(jmg, pose, 0.1)) {
      RCLCPP_ERROR(get_logger(), "IK failed — skipping pose2.");
      return;
    }

    std::vector<double> ik_joints;
    ik.copyJointGroupPositions(jmg, ik_joints);

    std::vector<double> full;
    full.push_back(start_joints_[0]);
    full.insert(full.end(), ik_joints.begin(), ik_joints.end());

    add_motion("move_to_pose2_left", full);
  }

  // ======================================================================
  // RUN PLAYMOTION SEQUENCE
  // ======================================================================
  void run_sequence()
  {
    send_motion("move_to_start_left");
  }

  void send_motion(const std::string & key)
  {
    if (!pm_client_) return;

    auto goal = PlayMotion2::Goal();
    goal.motion_name = key;
    goal.skip_planning = false;

    pm_client_->async_send_goal(goal,
      rclcpp_action::Client<PlayMotion2>::SendGoalOptions());
  }

  // ======================================================================
  // MEMBERS
  // ======================================================================
  Offset offset_;
  bool robot_description_received_ = false;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp_action::Client<PlayMotion2>::SharedPtr pm_client_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  std::vector<double> start_joints_;
};

// ======================================================================
// MAIN
// ======================================================================
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  Offset off;

  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    if (a[0]=='x') off.dx = std::stod(a.substr(1));
    if (a[0]=='y') off.dy = std::stod(a.substr(1));
    if (a[0]=='z') off.dz = std::stod(a.substr(1));
  }

  rclcpp::spin(std::make_shared<PickAndPlaceClient>(off));
  rclcpp::shutdown();
  return 0;
}

