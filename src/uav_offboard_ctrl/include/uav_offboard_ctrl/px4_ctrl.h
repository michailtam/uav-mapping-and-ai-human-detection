#ifndef FCU_CONTROL_H
#define FCU_CONTROL_H

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/position_setpoint_triplet.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/home_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <px4_msgs/msg/battery_status.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace std;


struct PosInENU {
    /**
     * @brief Stores the position ENU coordinates (for Gazebo).
     */
    double x, y, z, yaw;
};

struct PosInNED {
    /**
     * @brief Stores the position NED coordinates (for PX4).
     */
    double x, y, z, yaw;
};

struct PosInWGS84 {
    /**
     * @brief Stores position WGS84 coordinates.
     */
    double lat, lon, alt;
};

struct Velocity {
    /**
     * @brief Stores the linear and angular velocity
     */
    float linear_x, linear_y, linear_z, angular_z;
};

struct ScanDistance {
    /**
     * @brief Stores the front, right, back and left distances (if obstacle detected)
     */
    float front, right, back, left;
};


class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl();
    void engageOffboardMode();
    void arm();
    void disarm();
    void takeOff();
    void hover();
    void land();
    void RTL();
    void teleop();
    void run();
    void localPoseCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void globalPoseCallback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
    void statusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    void vehicleAttitudeCallback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg);
    void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void teleopArmedCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void batteryStatusCallback(const px4_msgs::msg::BatteryStatus::SharedPtr msg);
    
private:
    void publishOffboardControlMode();
    void publishTrajectorySetpoint(float pos_x, float pos_y, float pos_z, float pos_yaw);
    void sendVehicleCommand(uint16_t command, float param1, float param2);
    void srvCallback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future);
    void homePoseCallback(px4_msgs::msg::HomePosition::SharedPtr msg);
    void goalPoseCallback(px4_msgs::msg::PositionSetpointTriplet::SharedPtr msg);
    void timerUpdateStateMachine(void);

    enum class State{
        INIT_MODE,              // Initialization Mode
		OFFBOARD_MODE,          // Flight Mode
		ARM_MODE,               // Flight Mode
        DISARM_MODE,            // Flight Mode
        TAKEOFF_MODE,           // Flight Mode
        HOVER_MODE,             // Flight Mode
        POSITION_MODE,          // Flight Mode    
        LAND_MODE,              // Flight Mode
        RTL_MODE,               // Flight Mode (Return To Land)
        TELEOP_MODE             // Teleoperation Mode
	} state_;

	uint8_t service_result_;
	bool service_done_;
    float take_off_height_;
	rclcpp::TimerBase::SharedPtr timer_;
    std::vector<PosInENU> trajectory_;
    size_t trajectory_index_ = 0;
    bool msg_logged_ = false;
    bool teleop_armed_ = false;
    float true_yaw_ = 0.0;  // Current yaw value of drone
    float cmd_yaw_ = 0.0;    // The yaw value send as command
    float battery_status_perc_;  // Saves the battery status in percent
    bool key_pressed_ = false; // Determines if a key for velocity was pressed by the teleop node

    px4_msgs::msg::VehicleStatus::SharedPtr vehicle_status_msg_;
    
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_ctrl_mode_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr traj_setpoint_pub_;
    rclcpp::Subscription<px4_msgs::msg::HomePosition>::SharedPtr uav_home_pos_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr uav_local_pos_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr uav_global_pos_sub_;
    rclcpp::Subscription<px4_msgs::msg::PositionSetpointTriplet>::SharedPtr uav_target_pos_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr uav_status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr uav_attitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr battery_status_sub_;
    
    // Twist message subscribers for teleop
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr uav_velocity_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr teleop_armed_sub_;
    
    // Service for vehicle commands
    rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_cmd_client_;   
    
    PosInENU curr_loc_;          // Local position coordinates in NED
    PosInWGS84 curr_glob_;       // Global position coordinates in WGS84
    PosInENU target_loc_;        // Local target position coordinates in NED
    PosInWGS84 target_glob_;     // Global target position coordinates in NED
    PosInENU home_loc_;          // Local home position coordinates in NED
    PosInWGS84 home_glob_;       // Global home position coordinates in NED
    PosInENU hover_pos_loc_;     // Fixed hover position
    Velocity velocity_;          // The linear and angular velocities to publish to PX4
};

#endif