#include "uav_offboard_ctrl/px4_ctrl.h"


OffboardControl::OffboardControl() : Node("offboard_ctrl"), 
    /**
    * @brief 
    */
    state_{State::INIT_MODE},   // Initial state at startup
    service_result_{0},         // State of the service request
    service_done_{false},       // Flag that indicates if the service request has succeeded or not  
    take_off_height_{-5.0}      // NED Coord system i.e. z is pointing downwards but in ROS is facing upwards
    {
    
    // Create the service for changing the control mode
    RCLCPP_INFO(this->get_logger(), "Starting Offboard Control with PX4 services");
    vehicle_cmd_client_ = this->create_client<px4_msgs::srv::VehicleCommand>("/fmu/vehicle_command");

    // Before proceeding wait until the service is available
    while (!vehicle_cmd_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::timerUpdateStateMachine, this)); // 10 Hz

    // To correctly receive data, set the quality of service
    rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));

    // Setting the QoS parameters
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
    qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    qos_profile.keep_last(10);

    // ** Publishers ** 
    // Create the offboard control mode and trajectory publisher
    offboard_ctrl_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    traj_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    
    // ** Subscribers **
    // Callback to store the current position of the UAV
    uav_home_pos_sub_ = this->create_subscription<px4_msgs::msg::HomePosition>("/fmu/out/home_position_v1", qos_profile, 
            std::bind(&OffboardControl::homePoseCallback, this, std::placeholders::_1));
    uav_local_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position_v1", qos_profile,  // QoS of 10
            std::bind(&OffboardControl::localPoseCallback, this, std::placeholders::_1));
    uav_global_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>("/fmu/out/vehicle_global_position", qos_profile,  // QoS of 10
            std::bind(&OffboardControl::globalPoseCallback, this, std::placeholders::_1));
    uav_target_pos_sub_ = this->create_subscription<px4_msgs::msg::PositionSetpointTriplet>("/fmu/out/position_setpoint_triplet", qos_profile, 
            std::bind(&OffboardControl::goalPoseCallback, this, std::placeholders::_1));
    uav_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status_v1", qos_profile, 
            std::bind(&OffboardControl::statusCallback, this, std::placeholders::_1));
}

void OffboardControl::srvCallback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future) {
    /**
     * @brief Callback function which waits until the service has completed with "Command accepted", 
     * this means that the requested command was executed.
     */
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready) 
    {
        auto reply = future.get()->reply;
        uint8_t service_result_ = reply.result;
        RCLCPP_INFO(this->get_logger(), "Service replied successfully with %d", service_result_);

        switch (service_result_)
		{
		case reply.VEHICLE_CMD_RESULT_ACCEPTED:
			RCLCPP_INFO(this->get_logger(), "Command accepted");
			break;
		case reply.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
			RCLCPP_WARN(this->get_logger(), "Command temporarily rejected");
			break;
		case reply.VEHICLE_CMD_RESULT_DENIED:
			RCLCPP_WARN(this->get_logger(), "Command denied");
			break;
		case reply.VEHICLE_CMD_RESULT_UNSUPPORTED:
			RCLCPP_WARN(this->get_logger(), "Command unsupported");
			break;
		case reply.VEHICLE_CMD_RESULT_FAILED:
			RCLCPP_WARN(this->get_logger(), "Command failed");
			break;
		case reply.VEHICLE_CMD_RESULT_IN_PROGRESS:
			RCLCPP_WARN(this->get_logger(), "Command in progress");
			break;
		case reply.VEHICLE_CMD_RESULT_CANCELLED:
			RCLCPP_WARN(this->get_logger(), "Command cancelled");
			break;
		default:
			RCLCPP_WARN(this->get_logger(), "Command reply unknown");
			break;
		}
        service_done_ = true;
    } 
    else {
        RCLCPP_INFO(this->get_logger(), "Service In-Progress...");  
    }
}

void OffboardControl::engageOffboardMode() {
    /**
     * @brief Send the appropriate command to switch to offboard mode. 
     * This mode is required to handle the drone via code.
     */
    RCLCPP_INFO(this->get_logger(), "Requesting switch to Offboard mode");
	sendVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
}

void OffboardControl::publishOffboardControlMode() {
    /**
     * @brief Change to offboard mode with the desired parameters.
     */
    OffboardControlMode mode{};
    mode.position = true;
    mode.velocity = true;
    mode.acceleration = false;
    mode.attitude = false;
    mode.body_rate = false;
    mode.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_ctrl_mode_pub_->publish(mode);
}

void OffboardControl::publishTrajectorySetpoint(float pos_x, float pos_y, float pos_z, float pos_yaw) {
    /**
     * @brief Publish a trajectory setpoint (i.e. the position to fly).
     */
    TrajectorySetpoint msg{};
	msg.position = {pos_x, pos_y, pos_z};
	msg.yaw = pos_yaw;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	traj_setpoint_pub_->publish(msg);
}

void OffboardControl::sendVehicleCommand(uint16_t command, float param1, float param2) {
    /**
     * @brief Publish vehicle commands
     * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
     * @param param1    Command parameter 1
     * @param param2    Command parameter 2
     */
    auto req = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();

    // Populate the command parameters to send
	VehicleCommand cmd{};
	cmd.param1 = param1;
	cmd.param2 = param2;
	cmd.command = command;
	cmd.target_system = 1;
	cmd.target_component = 1;
	cmd.source_system = 1;
	cmd.source_component = 1;
	cmd.from_external = true;
	cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	req->request = cmd;

    // Callback function, which is triggered when the service response is ready
	service_done_ = false;
    auto result = vehicle_cmd_client_->async_send_request(req, std::bind(&OffboardControl::srvCallback, 
        this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Command send");
}

void OffboardControl::timerUpdateStateMachine(void) {
    /**
     * @brief Finite-State-Machine (FSM) to switch from state to state in specific intervals.
     */
    static uint8_t num_of_steps = 0;  // Counter to engage the offboard mode

	// Note: Always publish OffboardControlMode
	publishOffboardControlMode();

	switch (state_)
	{
	case State::INIT_MODE:
        engageOffboardMode();
		state_ = State::OFFBOARD_MODE;
		break;
	case State::OFFBOARD_MODE:
        if(service_done_) {
            // Wait 10 steps (warming up offboard mode) before engaging the arm mode
            if (++num_of_steps > 10) {
                RCLCPP_INFO(this->get_logger(), "Entered offboard mode");
                RCLCPP_INFO(this->get_logger(), "Arming mode engaged");
                state_ = State::ARMING_MODE;
                arm();
            } 
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to enter offboard mode, exiting");
            rclcpp::shutdown();
        } 
		break;
    case State::ARMING_MODE:
        if(service_done_) {
			if (service_result_==0) {
                RCLCPP_INFO(this->get_logger(), "Vehicle armed");
                RCLCPP_INFO(this->get_logger(), "Take Off engaged");
                state_ = State::TAKEOFF_MODE;
			} 
		} else {
            RCLCPP_ERROR(this->get_logger(), "Arming failed, exiting!");
            rclcpp::shutdown();
        }
        break;
    case State::TAKEOFF_MODE:
        takeOff();
        if(!service_done_) {
            RCLCPP_ERROR(this->get_logger(), "Take Off failed, exiting!");
            rclcpp::shutdown();  
        } 
        break;
    case State::HOVER_MODE:
        if(service_done_) {
            if(service_result_==0) {
                if (!msg_logged_) {
                    RCLCPP_INFO(this->get_logger(), "Hover at global position:%f %f %f", 
                        curr_glob_.lat, curr_glob_.lon, curr_glob_.alt);
                    msg_logged_ = true;
                }
                hover();
            } 
        } else {
            RCLCPP_ERROR(this->get_logger(), "Hover mode failed, exiting!");
            rclcpp::shutdown();
        }
        break;
    case State::TELEOP_MODE:
        if (!msg_logged_) {
            RCLCPP_INFO(this->get_logger(), "Teleop mode");
            msg_logged_ = true;
        }
        break;
    case State::NAVIGATION_MODE:
        if(service_done_) {
            if(service_result_==0) {
                if (!msg_logged_) {
                    RCLCPP_INFO(this->get_logger(), "Navigation mode engaged");
                }
            } 
        } else {
            RCLCPP_ERROR(this->get_logger(), "Navigation mode failed, exiting!");
            rclcpp::shutdown();
        } 
        break;
    case State::LANDING_MODE:
        land();
        if(service_done_) {
            if(service_result_==0) {
                    RCLCPP_INFO(this->get_logger(), "Landing completed");
                    RCLCPP_INFO(this->get_logger(), "Disarming mode engaged");
                    state_ = State::DISARMING_MODE;
                    disarm();
            } 
        } else {
            RCLCPP_ERROR(this->get_logger(), "Landing mode failed, exiting!");
            rclcpp::shutdown();
        }
        break;
    case State::DISARMING_MODE:
        if(service_done_) {
            if(service_result_==0) {
                RCLCPP_INFO(this->get_logger(), "Vehicle disarmed!");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Disarming mode failed, exiting!");
            rclcpp::shutdown();
        }
        break;
	default:
        if (!msg_logged_) {
            RCLCPP_INFO(this->get_logger(), "Unknown state mode!");
            msg_logged_ = true;
        }
		break;
	}
}

void OffboardControl::arm() {
    /**
     * @brief Publish the command to arm the vehicle
     */
    sendVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 
        VehicleCommand::ARMING_ACTION_ARM, 0.0);  // param 2 = 0.0
}

void OffboardControl::disarm() {
    /**
     * @brief Publish the command to disarm the vehicle
     */
    sendVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 
        VehicleCommand::ARMING_ACTION_DISARM, 0.0); // param 2 = 0.0
}

void OffboardControl::takeOff() {
    /**
     * @brief Publish the command for take off. When the desired altitude has been reached, 
     * hover on position and change the state to HOVER_MODE.
     */

    // Takeoff until approx. 5 m altitude has been reached
    if (curr_loc_.z >= (take_off_height_+0.1)) {
        publishTrajectorySetpoint(curr_loc_.x, curr_loc_.y, take_off_height_, curr_loc_.yaw);
    } else {
        RCLCPP_INFO(this->get_logger(), "Takeoff altitude of %.2fm reached", (-1)*take_off_height_);
        RCLCPP_INFO(this->get_logger(), "Hover mode engaged");
        state_ = State::HOVER_MODE;
        hover_pos_loc_.x = curr_loc_.x;
        hover_pos_loc_.y = curr_loc_.y;
        hover_pos_loc_.z = curr_loc_.z;
        hover_pos_loc_.yaw = curr_loc_.yaw;

        msg_logged_ = false;
    }
}

void OffboardControl::hover() {
    /**
     * @brief The drone hovers on position.
     */
    publishTrajectorySetpoint(hover_pos_loc_.x, hover_pos_loc_.y, hover_pos_loc_.z, hover_pos_loc_.yaw);
}

void OffboardControl::land() {
    /**
     * @brief The landing process gets fully taken charged by PX4 and not from the program 
     */
    sendVehicleCommand(VehicleCommand::VEHICLE_CMD_NAV_LAND, 0.0, 0.0);
}

void OffboardControl::localPoseCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    /**
    * @brief Save the current local position of the UAV (i.e. NED).
    */
    curr_loc_.x = msg->x;
    curr_loc_.y = msg->y;
    curr_loc_.z = msg->z;
    curr_loc_.yaw = msg->heading;

    // RCLCPP_INFO(this->get_logger(), "Local pos: x:%f y:%f z:%f, yaw:%f", 
    //     curr_loc_.x, curr_loc_.y, curr_loc_.z, curr_loc_.yaw);
}

void OffboardControl::globalPoseCallback(const px4_msgs::msg::VehicleGlobalPosition::SharedPtr msg) {
    /**
    * @brief Save the current global position of the UAV (i.e. WGS84).
    */
    curr_glob_.lat = msg->lat;
    curr_glob_.lon = msg->lon;
    curr_glob_.alt = msg->alt;

    // RCLCPP_INFO(this->get_logger(), "Global pos: lat:%f lon:%f alt:%f", 
    //     curr_glob_.lat, curr_glob_.lon, curr_glob_.alt);
}

void OffboardControl::homePoseCallback(const px4_msgs::msg::HomePosition::SharedPtr msg) {
    /** 
    * @brief Save the local/global home position and send it (local pos) to Nav2. 
    */
    home_loc_.x = msg->x;
    home_loc_.y = msg->y;
    home_loc_.z = msg->z;
    home_glob_.lat = msg->lat;
    home_glob_.lon = msg->lon;
    home_glob_.alt = msg->alt;   

    // RCLCPP_INFO(this->get_logger(), "Global home position lat:%f lon:%f alt:%f set", 
    //     home_glob_.lat, home_glob_.lon, home_glob_.alt);
}

void OffboardControl::goalPoseCallback(const px4_msgs::msg::PositionSetpointTriplet::SharedPtr msg) {
    /**
    * @brief Save the global goal position, convert it to ENU and send it (local pos) to Nav2.
    */

    if (state_ == State::HOVER_MODE) 
    {
        target_glob_.lat = msg->current.lat;
        target_glob_.lon = msg->current.lon;
        target_glob_.alt = msg->current.alt;
    
        RCLCPP_INFO(this->get_logger(), "Global goal position lat:%f lon:%f alt:%f set and send to Nav2", 
            home_glob_.lat, home_glob_.lon, home_glob_.alt);
    }
}

void OffboardControl::statusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
    /**
     * @brief Saves the vehicle status.
     */
    vehicle_status_msg_ = msg;
}

void OffboardControl::run() {
    /**
     * @brief Start the program execution loop.
     */
    rclcpp::spin(shared_from_this());
}


// Program execution
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OffboardControl>();
  node->run();

  rclcpp::shutdown();
  return 0;
}