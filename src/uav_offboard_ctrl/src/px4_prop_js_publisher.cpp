#include "uav_offboard_ctrl/px4_propeller_jointstate_pub.h"


Px4PropellerJointStatePub::Px4PropellerJointStatePub() : Node("px4_propeller_jointstate_publisher"),
    joint_names_{
      "front_prop_ccw_joint",
      "back_prop_ccw_joint",
      "front_prop_cw_joint",
      "back_prop_cw_joint"},
    angles_{0.0, 0.0, 0.0, 0.0},    // Initial joint angles
    armed_(false)
  { 
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // Use SensorData QoS (best effort) because PX4 publishes actuator outputs
    // with a similar QoS profile. Using default Reliable QoS can cause missed messages.
    auto qos = rclcpp::SensorDataQoS();

    // Publisher for /joint_states (RViz uses this via robot_state_publisher)
    motor_sub_ = this->create_subscription<px4_msgs::msg::ActuatorMotors>("/fmu/out/actuator_motors", qos,
      std::bind(&Px4PropellerJointStatePub::motorCallback, this, std::placeholders::_1));

    // Subscribe to PX4 motor outputs (normalized values 0..1)
    status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status_v1", qos,
      std::bind(&Px4PropellerJointStatePub::statusCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Px4PropellerJointStatePub started (real motors, armed gating).");
  }

void Px4PropellerJointStatePub::statusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
    using px4_msgs::msg::VehicleStatus;
    armed_ = (msg->arming_state == VehicleStatus::ARMING_STATE_ARMED);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,"arming_state = %u (armed=%s)",
        msg->arming_state, armed_ ? "true" : "false");
}

void Px4PropellerJointStatePub::motorCallback(const px4_msgs::msg::ActuatorMotors::SharedPtr msg)
{
    /**
     * @brief Read PX4 arming_state and determine whether the props are allowed to spin.
     */
    // dt per message (approximate, purely visual)
    const double dt = 0.02;
    const double scale = 300.0;  // Normalized 1.0 -> 300 rad/s

    std::array<double, 4> speeds{0.0, 0.0, 0.0, 0.0};
    for (size_t i = 0; i < speeds.size() && i < msg->control.size(); ++i) {
        double v = static_cast<double>(msg->control[i]);
        if (std::isfinite(v)) {
        speeds[i] = v * scale;
        } else {
        speeds[i] = 0.0;
        }
    }

    // If not armed, force speeds to zero → no spin, even if PX4 publishes junk
    if (!armed_) {
        speeds = {0.0, 0.0, 0.0, 0.0};
    }

    // Apply rotation direction (CW vs CCW)
    // Assuming: index 0,1 are CCW, index 2,3 are CW
    speeds[2] = -speeds[2];  // CW rotates opposite
    speeds[3] = -speeds[3];  // CW rotates opposite

    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 500,
        "armed=%s motors(norm): [%.3f %.3f %.3f %.3f]",
        armed_ ? "true" : "false",
        (double)msg->control[0],
        (double)msg->control[1],
        (double)msg->control[2],
        (double)msg->control[3]);

    // Integrate only armed speeds; when disarmed, speeds=0 so angles freeze
    for (size_t i = 0; i < angles_.size(); ++i) {
        angles_[i] += speeds[i] * dt;
        angles_[i] = std::fmod(angles_[i], 2.0 * M_PI);
    }

    sensor_msgs::msg::JointState js;
    js.header.stamp = this->now();
    js.name.assign(joint_names_.begin(), joint_names_.end());
    js.position.assign(angles_.begin(), angles_.end());
    js.velocity.assign(speeds.begin(), speeds.end());

    joint_pub_->publish(js);
}


// Program execution
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Px4PropellerJointStatePub>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
