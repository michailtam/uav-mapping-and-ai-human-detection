#ifndef FCU_CONTROL_H
#define FCU_CONTROL_H

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
    FCUControl();
    void sendCommand(uint16_t command, float param1, float param2);
    void srvCallback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future);
    void publishOffBoardCtrlMode();
    void arm();
    void disarm();

private:
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboardCtrlModePub_;
    rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr cmdClient_;
    std::atomic<bool> action_done_{false};

    float curr_x_;
    float curr_y_;
    float curr_z_;
};

#endif