#include "px4_ctrl.h"


OffboardControl::OffboardControl() : Node("offboard_ctrl"), state_{State::init}, service_result_{0}, service_done_{false} {

    // Create the offboard control mode and trajectory publisher
    offboard_ctrl_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    traj_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    
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

    timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::timerUpdateStateMachine, this));

    // To receive the estimated position in a local frame in Cartesian space, 
    // to correctly receive the topic, set the quality of service first
    rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));

    // Setting the QoS parameters
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
    qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
    qos_profile.keep_last(10);

    // Callback to store the current position of the UAV
    uav_pose_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position_v1", qos_profile,  // QoS of 10
            std::bind(&OffboardControl::poseCallback, this, std::placeholders::_1));
}

void OffboardControl::switchToOffboardMode() {
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
	msg.yaw = 0.0;
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
    static uint8_t num_of_steps = 0;

	// Offboard_control_mode needs to be paired with trajectory_setpoint
	publishOffboardControlMode();
	// publishTrajectorySetpoint();

	switch (state_)
	{
	case State::init :
		switchToOffboardMode();
		state_ = State::offboard_requested;
		break;
	case State::offboard_requested :
		if(service_done_){
			if (service_result_==0){
				RCLCPP_INFO(this->get_logger(), "Entered offboard mode");
				state_ = State::wait_for_stable_offboard_mode;				
			} else {
				RCLCPP_ERROR(this->get_logger(), "Failed to enter offboard mode, exiting");
				rclcpp::shutdown();
			}
		}
		break;
	case State::wait_for_stable_offboard_mode :
		if (++num_of_steps>10){
			arm();
			state_ = State::arm_requested;
		}
		break;
	case State::arm_requested :
		if(service_done_){
			if (service_result_==0){
				RCLCPP_INFO(this->get_logger(), "Vehicle is armed");
				state_ = State::armed;
			} else {
				RCLCPP_ERROR(this->get_logger(), "Failed to arm, exiting");
				rclcpp::shutdown();
			}
		}
		break;
    case State::armed:
        RCLCPP_INFO(this->get_logger(), "Take Off");
        state_ = State::takeoff;
        break;
    case State::takeoff:
        takeOff();
        break;
    case State::position_reached:
        break;
	default:
		break;
	}
}

void OffboardControl::arm() {
  RCLCPP_INFO(this->get_logger(), "Arm");	
  sendVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, VehicleCommand::ARMING_ACTION_ARM, 0.0);
}

void OffboardControl::disarm() {
  RCLCPP_INFO(this->get_logger(), "Disarm");
  sendVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, VehicleCommand::ARMING_ACTION_DISARM, 0.0);
}

void OffboardControl::takeOff() {
    publishTrajectorySetpoint(0.0, 0.0, -5.0);  // Takeoff until 5 m has reached.
}

// void OffboardControl::flyTo(float x, float y, float z) {
//     /**
//      * @brief Move the UAV to the desired position.
//      */
//     Point3D start = {curr_x_, curr_y_, curr_z_};  
//     Point3D end = {x, y, z}; 
//     double v_max = 0.5; // 2 meters per second
//     double time_step = 1.0/10.0; 
//     std::vector<Point3D> trajectory = planTrajectory(start, end, v_max, time_step);
//     px4_msgs::msg::TrajectorySetpoint msg{};
//     rclcpp::Rate loop_rate(50);     // 50 Hz

//     for (const auto& point : trajectory) {
//         msg.position = {point.x, point.y, point.z};
//         msg.yaw = 0.0; 
//         msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
//         traj_setpoint_pub_->publish(msg);
//         loop_rate.sleep();
//     }
// }

void OffboardControl::poseCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    /**
    * @brief Save the current coordinates of the UAV.
    */
    curr_x_ = msg->x;
    curr_y_ = msg->y;
    curr_z_ = msg->z;
    // RCLCPP_INFO(this->get_logger(), "Pose: x:%f y:%f z:%f", curr_x_, curr_y_, curr_z_);
}

double OffboardControl::calculateDistance(const Point3D& start, const Point3D& end) {
    /**
    * @brief Calculate the Euclidean distance between two 3D points.
    */
    return std::hypot(end.x - start.x, end.y - start.y, end.z - start.z);
}

std::vector<Point3D> OffboardControl::planTrajectory(const Point3D& start, const Point3D& end, double v_max, double time_step) {
    /**
     * @brief Plan the trajectory between two points
     */
    std::vector<Point3D> waypoints;
    
    // Calculate the total distance between the start and end points
    double total_distance = calculateDistance(start, end);

    // Calculate the total time required to reach the end point
    double total_time = total_distance / v_max;

    // Calculate the number of time steps needed
    int num_steps = std::ceil(total_time / time_step);

    // Calculate the velocity components for each axis
    double v_x = (end.x - start.x) / (num_steps * time_step);
    double v_y = (end.y - start.y) / (num_steps * time_step);
    double v_z = (end.z - start.z) / (num_steps * time_step);

    // Generate waypoints for each time step
    for (int i = 0; i <= num_steps; ++i) {
        double t = i * time_step;  // Current time

        // Calculate the position at time t
        Point3D waypoint;
        waypoint.x = start.x + v_x * t;
        waypoint.y = start.y + v_y * t;
        waypoint.z = start.z + v_z * t;

        waypoints.push_back(waypoint);
    }

    return waypoints;
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