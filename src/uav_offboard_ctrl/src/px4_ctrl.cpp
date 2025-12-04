#include "uav_offboard_ctrl/px4_ctrl.h"


OffboardControl::OffboardControl() : Node("offboard_ctrl"), 
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

    // To receive the estimated position in a local frame in Cartesian space, 
    // to correctly receive the topic, set the quality of service first
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
    // nav2_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10); // TODO: Check the topic name
    
    // ** Subscribers **
    // Callback to store the current position of the UAV
    uav_pose_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position_v1", qos_profile,  // QoS of 10
            std::bind(&OffboardControl::localPositionCallback, this, std::placeholders::_1));
    uav_target_pose_sub_ = this->create_subscription<px4_msgs::msg::PositionSetpointTriplet>("/fmu/out/position_setpoint_triplet", qos_profile, 
            std::bind(&OffboardControl::targetPositionCallback, this, std::placeholders::_1));
    uav_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status_v1", qos_profile, 
            std::bind(&OffboardControl::statusCallback, this, std::placeholders::_1));
}

void OffboardControl::engageOffboardMode() {
    /**
     * @brief Send a command to switch to offboard mode 
     */
    RCLCPP_INFO(this->get_logger(), "Requesting switch to Offboard mode");
	sendVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
}

void OffboardControl::srvCallback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future) {
    /**
     * @brief Callback function which waits for the service to return with a value. When it returns, 
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

void OffboardControl::publishOffboardControlMode() {
    /**
     * @brief Is called to change to offboard mode. Here, position and altitude controls are active
     */
    OffboardControlMode mode{};
    mode.position = true;
    mode.velocity = false;
    mode.acceleration = false;
    mode.attitude = false;
    mode.body_rate = false;
    mode.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_ctrl_mode_pub_->publish(mode);
}

void OffboardControl::publishTrajectorySetpoint(float pos_x, float pos_y, float pos_z) {
    /**
     * @brief Publish a trajectory setpoint (i.e. the position to fly)
     */
    TrajectorySetpoint msg{};
	msg.position = {pos_x, pos_y, pos_z};
	msg.yaw = 0.0;  // Keep the drone only facing forward.
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
     * @brief Finite-State-Machine (FSM) to switch from one state to the other in specific intervals.
     */
    static uint8_t num_of_steps = 0;  // Counter to engage the offboard mode

	// Always publish OffboardControlMode
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
                RCLCPP_INFO(this->get_logger(), "Arming mode engaged");
                state_ = State::ARMING_MODE;
                arm();
            } else if (service_result_==0) {
                RCLCPP_INFO(this->get_logger(), "Entered offboard mode");
                msg_logged_ = true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Switching to Offboard mode failed, exiting!");
                rclcpp::shutdown();
            }
        }
        else {
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
			} else {
                RCLCPP_ERROR(this->get_logger(), "Arming failed, exiting!");
                rclcpp::shutdown();
            }
		}
        break;
    case State::TAKEOFF_MODE:
        takeOff();
        if(service_done_) {
            if(service_result_==0) {
                if (!msg_logged_) {
                    RCLCPP_INFO(this->get_logger(), "Nav2 navigation mode engaged");
                    msg_logged_ = true;
                }
            } 
        } else {
            RCLCPP_ERROR(this->get_logger(), "Take Off failed, exiting!");
            rclcpp::shutdown();
        }
        break;
    case State::NAV2_NAV_MODE:
        if(service_done_) {
            if(service_result_==0) {
                if (!msg_logged_) {
                    RCLCPP_INFO(this->get_logger(), "Nav2 navigation mode...");
                    msg_logged_ = true;
                }
            } 
        } else {
            RCLCPP_ERROR(this->get_logger(), "Nav2 navigation mode failed, exiting!");
            rclcpp::shutdown();
        }
        break;
    case State::TELEOP_MODE:
        if (!msg_logged_) {
            RCLCPP_INFO(this->get_logger(), "Teleop mode");
            msg_logged_ = true;
        }
        break;
    case State::HOLD_MODE:
        if(service_done_) {
            if(service_result_==0) {
                RCLCPP_INFO(this->get_logger(), "Holding on position:%f %f %f", curr_x_, curr_y_, curr_z_);
                msg_logged_ = true;
            } 
        } else {
            RCLCPP_ERROR(this->get_logger(), "Holding mode failed, exiting!");
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
            } 
        } else {
            RCLCPP_ERROR(this->get_logger(), "Landing mode failed, exiting!");
            rclcpp::shutdown();
        }
        break;
    case State::DISARMING_MODE:
        disarm();
        if(service_done_) {
            if(service_result_==0) {
                RCLCPP_INFO(this->get_logger(), "Vehicle disarmed!");
                msg_logged_ = true;
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
  sendVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 
    VehicleCommand::ARMING_ACTION_ARM, 0.0);  // param 2 = 0.0
}

void OffboardControl::disarm() {
  sendVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 
    VehicleCommand::ARMING_ACTION_DISARM, 0.0); // param 2 = 0.0
}

void OffboardControl::takeOff() {
    // Takeoff until approx. 5 m altitude has been reached
    if (curr_z_ >= (take_off_height_+0.2)) {
        publishTrajectorySetpoint(curr_x_, curr_y_, take_off_height_);
    } else {
        RCLCPP_INFO(this->get_logger(), "Takeoff altitude of %.2fm reached", (-1)*take_off_height_);
        msg_logged_ = false;
        state_ = State::NAV2_NAV_MODE;
    }
}

void OffboardControl::land() {
    /**
     * @brief The landing process gets fully taken charge from PX4 and not from the program 
     */
    sendVehicleCommand(VehicleCommand::VEHICLE_CMD_NAV_LAND, 0.0, 0.0);
}

void OffboardControl::localPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    /**
    * @brief Save the current coordinates (PX4’s NED frame) of the UAV.
    */
    curr_x_ = msg->x;
    curr_y_ = msg->y;
    curr_z_ = msg->z;
    
    // Check if home position is set
    if (!home_position_set_) {
        home_x_ = curr_x_;
        home_y_ = curr_y_;
        home_z_ = curr_z_;
        home_position_set_ = true;
        RCLCPP_INFO(this->get_logger(), "Home pos: x:%f y:%f z:%f", home_x_, home_y_, home_z_);
    }
    // RCLCPP_INFO(this->get_logger(), "Local pos: x:%f y:%f z:%f", curr_x_, curr_y_, curr_z_);
}

void OffboardControl::targetPositionCallback(const px4_msgs::msg::PositionSetpointTriplet::SharedPtr msg) {
    /**
    * @brief Save the target coordinates (PX4’s NED frame) acquired from Nav2.
    */
    // target_x_ = msg->previous;
    // target_y_ = msg->current;
    // target_z_ = msg->next;
    RCLCPP_INFO(this->get_logger(), "Target pos: x:%f y:%f z:%f", msg->current.lat, msg->current.lon, msg->current.alt);
}

void OffboardControl::statusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
    /**
     * @brief Saves the vehicle status
     */
    vehicle_status_msg_ = msg;
}

void OffboardControl::run() {
    /**
     * @brief Start the program execution loop
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