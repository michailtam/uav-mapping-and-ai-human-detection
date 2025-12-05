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
#include <px4_msgs/srv/vehicle_command.hpp>
#include <GeographicLib/LocalCartesian.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace std;


struct PosInENU {
    /**
     * @brief Stores the x, y, and z ENU (i.e. Gazebo) coordinates.
     */
    double x, y, z;
};

struct PosInWGS84 {
    /**
     * @brief Stores the lat, lon and alt WGS84 coordinates.
     */
    double lat, lon, alt;
};

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl();
    void engageOffboardMode();
    void arm();
    void disarm();
    void takeOff();
    void land();
    void run();
    void homePositionCallback(const px4_msgs::msg::HomePosition::SharedPtr msg);
    void localPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void globalPositionCallback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg);
    void targetPositionCallback(const px4_msgs::msg::PositionSetpointTriplet::SharedPtr msg);
    void statusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
    PosInENU convertWGS84ToENU(double lat, double lon, double alt);
    
private:
    void publishOffboardControlMode();
    void publishTrajectorySetpoint(float pos_x, float pos_y, float pos_z);
    void sendVehicleCommand(uint16_t command, float param1, float param2);
    void srvCallback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future);
    void timerUpdateStateMachine(void);

    enum class State{
		INIT_MODE,              // initialization Mode
		OFFBOARD_MODE,          // Flight Mode
		ARMING_MODE,            // Flight Mode
        DISARMING_MODE,         // Flight Mode
        TAKEOFF_MODE,           // Flight Mode
        HOLD_MODE,              // Flight Mode
        POSITION_MODE,          // Flight Mode    
        LANDING_MODE,           // Flight Mode
        RETURN_MODE,            // Flight Mode
        TELEOP_MODE,            // Teleoperation Mode
        RTL_MODE,               // Automatic Flight Mode (Return to Land)
        NAV2_NAV_MODE           // Nav2 Navigation Mode (executes ROS2 Nav2 for navigation)
	} state_;

	uint8_t service_result_;
	bool service_done_;
    float take_off_height_;
	rclcpp::TimerBase::SharedPtr timer_;
    std::vector<PosInENU> trajectory_;
    size_t trajectory_index_ = 0;
    bool navigating_ = false;
    bool msg_logged_ = false;
    bool home_pos_set_ = false;
    bool target_pos_set_ = false;

    px4_msgs::msg::VehicleStatus::SharedPtr vehicle_status_msg_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_ctrl_mode_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr traj_setpoint_pub_;
    rclcpp::Subscription<px4_msgs::msg::HomePosition>::SharedPtr uav_home_pos_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr uav_pos_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr uav_pos_global_sub_;
    rclcpp::Subscription<px4_msgs::msg::PositionSetpointTriplet>::SharedPtr uav_target_pos_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr uav_status_sub_;
    rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_cmd_client_;
    
    
    PosInENU curr_loc_;          // Local position coordinates in NED
    PosInWGS84 curr_glob_;       // Global position coordinates in WGS84
    PosInENU target_loc_;        // Local target position coordinates in NED
    PosInWGS84 target_glob_;     // Global target position coordinates in NED
    PosInENU home_loc_;          // Local home position coordinates in NED
    PosInWGS84 home_glob_;       // Global home position coordinates in NED
};

#endif