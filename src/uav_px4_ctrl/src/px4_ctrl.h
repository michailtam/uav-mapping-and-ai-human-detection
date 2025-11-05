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
using namespace std;


struct Point3D {
    /**
     * @brief Store the x, y, and z coordinates.
     */
    double x, y, z;
};

class FCUControl : public rclcpp::Node
{
public:
    FCUControl();
    void sendCommand(uint16_t command, float param1, float param2);
    void srvCallback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future);
    void poseCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    std::vector<Point3D> planTrajectory(const Point3D& start, const Point3D& end, double v_max, double time_step);
    double calculateDistance(const Point3D& start, const Point3D& end);
    void publishOffBoardCtrlMode();
    void arm();
    void disarm();
    void takeOff();
    void flyTo(float x, float y, float z);
    void run();

private:
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboardCtrlModePub_;
    // rclcpp::Publisher<TrajectorySetpoint>::SharedPtr setpointPublisher_;
    rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr cmdClient_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr traj_cmd_pub_;
    std::atomic<bool> action_done_{false};
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr uavPoseSub_;
    
    float curr_x_;
    float curr_y_;
    float curr_z_;
};

#endif