#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;


class FCUControl : public rclcpp::Node
{
public:
    FCUControl() : Node("fcu_ctrl") {
        // Create the service for changing the control mode
        cmd_client = create_client<px4_msgs::srv::VehicleCommand>("/fmu/vehicle_command");
        // Publish a trigger to enable offboard control mode, publish the desired setpoint to reach
        trig_pub = create_publisher<px4_msgs::msg::TrajectorySetPoint>("/fmu/in/trajectory_setpoint", 10);
        offboard_ctrl_pub = create_publisher<px4_msgs::msgs::Off

    }
};