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
    void navigate();
    void run();
    void poseCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    // std::vector<Point3D> planTrajectory(const Point3D& start, const Point3D& end, double v_max, double time_step);  // TODO: move to another pkg
    // double calculateDistance(const Point3D& start, const Point3D& end); // TODO: move to another pkg
    // void prepareTrajectory(float x, float y, float z); // TODO: move to another pkg
    
private:
    void publishOffboardControlMode();
    void publishTrajectorySetpoint(float pos_x, float pos_y, float pos_z);
    void sendVehicleCommand(uint16_t command, float param1, float param2);
    void srvCallback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future);
    void timerUpdateStateMachine(void);

    enum class State{
		INIT_MODE,              // initialization Mode
		OFFBOARD_REQUESTED,     
		OFFBOARD_MODE,          // Flight Mode
		ARM_REQUESTED,          
        ARM_MODE,               // Flight Mode
        TAKEOFF_MODE,           // Flight Mode
        HOLD_MODE,              // Flight Mode
        MISSION_MODE,           // Flight Mode
        POSITION_MODE,          // Flight Mode    
        LAND_MODE,              // Flight Mode
        RETURN_MODE             // Flight Mode
	} state_;

	uint8_t service_result_;
	bool service_done_;
    float take_off_height_;
	rclcpp::TimerBase::SharedPtr timer_;
    Point3D target_position_; // Current position setpoint in NED
    std::vector<Point3D> trajectory_;
    size_t trajectory_index_ = 0;
    bool navigating_ = false;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_ctrl_mode_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr traj_setpoint_pub_;
    rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_cmd_client_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr uav_pose_sub_;
    
    float curr_x_;
    float curr_y_;
    float curr_z_;
};

#endif