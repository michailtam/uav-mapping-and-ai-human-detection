#include "px4_ctrl.h"


FCUControl::FCUControl() : Node("fcu_ctrl") {
    // Create the service for changing the control mode
    cmdClient_ = create_client<px4_msgs::srv::VehicleCommand>("/fmu/vehicle_command");
    // Publish a trigger to enable offboard control mode
    // trajectoryPub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    offboardCtrlModePub_ = create_publisher<px4_msgs::msg::OffboardControlMode>("fmu/in/offboard_control_mode", 10);

    // Before proceeding wait until the service is available
    while (!cmdClient_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }
    
    // To receive the estimated position in a local frame in Cartesian space, 
    // to correctly receive the topic, set the quality of service first
    rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));

    // Setting the QoS parameters
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
    qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);  // Depth could be set here
    qos_profile.keep_last(10);  
}

void FCUControl::srvCallback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future) {
    /*
    Callback function which waits for the service to return with a value. When
    it returns, this means that the requested command was executed.
    */
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready) {
        auto reply = future.get()->reply;
        uint8_t service_result_ = reply.result;
        RCLCPP_INFO(this->get_logger(), "Service replied successfully with %d", service_result_);  
    } 
    else {
        RCLCPP_INFO(this->get_logger(), "Service still in-progress...");  
    }
}

void FCUControl::publishOffBoardCtrlMode() {
    /*
    Specify the kinds of input data. For example, in this case enable the offboard control mode, 
    controlling the desired position. 
    NOTE: To avoid interruptions, start this as a thread at 1 Hz.
    */
    rclcpp::Rate loop_rate(1);
    
    while( rclcpp::ok() ) {
        OffboardControlMode msg{};
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    	offboardCtrlModePub_->publish(msg);
        loop_rate.sleep();
    }
}

void FCUControl::sendCommand(uint16_t command, float param1, float param2) {
    /**
     * @brief Publish vehicle commands
     * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
     * @param param1    Command parameter 1
     * @param param2    Command parameter 2
     */
    auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();

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
	request->request = cmd;

    // Callback function, which is triggered when the service response is ready
	auto result = cmdClient_->async_send_request(request, std::bind(&FCUControl::srvCallback, 
        this, std::placeholders::_1));
}

void FCUControl::arm() {
  RCLCPP_INFO(this->get_logger(), "Arming...");	
  sendCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, VehicleCommand::ARMING_ACTION_ARM, 0.0);
}

void FCUControl::disarm() {
  RCLCPP_INFO(this->get_logger(), "Disarming...");
  sendCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, VehicleCommand::ARMING_ACTION_DISARM, 0.0);
}

void FCUControl::takeOff() {
    /** 
    * @brief Switch on the offboard control mode (param2 = 6)
    */
    RCLCPP_INFO(this->get_logger(), "Take Off");
    sendCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    sleep(1);
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.position = {curr_x_, curr_y_, curr_z_-5.0}; // Take off to the given coords
    msg.yaw = 0.0; 
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    traj_cmd_pub_->publish(msg);
}

void FCUControl::flyTo(float x, float y, float z) {
    /**
     * @brief Move the UAV to the desired position.
     */
    Point3D start = {curr_x_, curr_y_, curr_z_};  
    Point3D end = {x, y, z}; 
    double v_max = 0.5; // 2 meters per second
    double time_step = 1.0/10.0; 
    std::vector<Point3D> trajectory = planTrajectory(start, end, v_max, time_step);
    px4_msgs::msg::TrajectorySetpoint msg{};
    rclcpp::Rate loop_rate(10);

    for (const auto& point : trajectory) {
        msg.position = {point.x, point.y, point.z};
        msg.yaw = 0.0; 
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        traj_cmd_pub_->publish(msg);
        loop_rate.sleep();
    }
}

double FCUControl::calculateDistance(const Point3D& start, const Point3D& end) {
    /**
    * @brief Calculate the Euclidean distance between two 3D points.
    */
    return std::hypot(end.x - start.x, end.y - start.y, end.z - start.z);
}

std::vector<Point3D> FCUControl::planTrajectory(const Point3D& start, const Point3D& end, double v_max, double time_step) {
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

void FCUControl::poseCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    /**
    * @brief Save the current coordinates of the UAV.
    */
    curr_x_ = msg->x;
    curr_y_ = msg->y;
    curr_z_ = msg->z;
}

void FCUControl::run() {
    rclcpp::spin(shared_from_this());
}


// Program execution
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FCUControl>();
  node->run();
  rclcpp::shutdown();
  return 0;
}