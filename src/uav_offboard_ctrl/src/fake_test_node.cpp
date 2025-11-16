#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class TestJointStatePub : public rclcpp::Node
{
public:
  TestJointStatePub()
  : Node("px4_propeller_jointstate_pub"),   // keep same node name
    joint_names_{
      "front_prop_ccw_joint",
      "back_prop_ccw_joint",
      "front_prop_cw_joint",
      "back_prop_cw_joint"
    },
    angles_{0.0, 0.0, 0.0, 0.0}
  {
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),      // 20 Hz
      std::bind(&TestJointStatePub::onTimer, this));

    RCLCPP_INFO(this->get_logger(), "TestJointStatePub started (fake spin).");
  }

private:
  void onTimer()
  {
    rclcpp::Time now = this->now();
    double dt = 0.05; // fixed 50 ms

    // constant angular velocity
    double omega = 20.0;  // rad/s (slow but visible)

    for (auto & a : angles_) {
      a += omega * dt;
      a = std::fmod(a, 2.0 * M_PI);
    }

    sensor_msgs::msg::JointState js;
    js.header.stamp = now;
    js.name.assign(joint_names_.begin(), joint_names_.end());
    js.position.assign(angles_.begin(), angles_.end());
    js.velocity = {omega, omega, omega, omega};

    joint_pub_->publish(js);
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::array<std::string, 4> joint_names_;
  std::array<double, 4> angles_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestJointStatePub>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
