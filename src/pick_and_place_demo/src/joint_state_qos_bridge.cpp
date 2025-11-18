#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class JointStateBridge : public rclcpp::Node
{
public:
    JointStateBridge()
    : Node("joint_state_qos_bridge")
    {
        // Subscribe with BEST_EFFORT (TIAGo)
        rclcpp::QoS sub_qos(rclcpp::KeepLast(10));
        sub_qos.best_effort();

        // Publish with RELIABLE (MoveIt requirement)
        rclcpp::QoS pub_qos(rclcpp::KeepLast(10));
        pub_qos.reliable();
        pub_qos.durability_volatile();

        RCLCPP_INFO(get_logger(), "⚡ QoS Bridge started");

        sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", sub_qos,
            [this](sensor_msgs::msg::JointState::SharedPtr msg)
            {
                pub_->publish(*msg);
            });

        pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states_best_effort", pub_qos);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStateBridge>());
    rclcpp::shutdown();
    return 0;
}
