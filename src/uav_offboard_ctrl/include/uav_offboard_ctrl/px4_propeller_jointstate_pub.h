#ifndef PX4_PROPELLER_JOINT_STATE_PUB_H
#define PX4_PROPELLER_JOINT_STATE_PUB_H

#include <array>
#include <cmath>
#include <string>
#include <vector>
#include <limits>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "px4_msgs/msg/actuator_motors.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"   // Type for vehicle_status_v1


/**
 * @brief This node publishes /joint_states for the 4 propeller joints, using REAL PX4 motor outputs 
 * from /fmu/out/actuator_motors. Propellers spin in RViz only when PX4 is ARMED. When disarmed to 
 * no motion (angles freeze).
 */
class Px4PropellerJointStatePub : public rclcpp::Node
{
public:
  Px4PropellerJointStatePub();

private:
  void statusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void motorCallback(const px4_msgs::msg::ActuatorMotors::SharedPtr msg);

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<px4_msgs::msg::ActuatorMotors>::SharedPtr motor_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;

  std::array<std::string, 4> joint_names_;
  std::array<double, 4> angles_;
  bool armed_;
};

#endif