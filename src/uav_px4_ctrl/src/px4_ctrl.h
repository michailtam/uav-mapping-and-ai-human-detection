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

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl();
    void switchToOffboardMode();
    void arm();
    void disarm();
    void takeOff();
    // void flyTo(float x, float y, float z);
    void run();
    void poseCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    std::vector<Point3D> planTrajectory(const Point3D& start, const Point3D& end, double v_max, double time_step);
    double calculateDistance(const Point3D& start, const Point3D& end);

private:
    void publishOffboardControlMode();
    void publishTrajectorySetpoint();
    void sendVehicleCommand(uint16_t command, float param1, float param2);
    void srvCallback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future);
    void timerUpdateStateMachine(void);

    enum class State{
		init,
		offboard_requested,
		wait_for_stable_offboard_mode,
		arm_requested,
		armed,
        takeoff,
        position_reached
	} state_;

	uint8_t service_result_;
	bool service_done_;
	rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_ctrl_mode_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr traj_setpoint_pub_;
    rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_cmd_client_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr uav_pose_sub_;
    
    float curr_x_;
    float curr_y_;
    float curr_z_;
};

#endif