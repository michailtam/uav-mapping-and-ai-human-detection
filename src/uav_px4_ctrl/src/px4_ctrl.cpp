#include "px4_ctrl.h"

using namespace std;


FCUControl::FCUControl() : Node("fcu_ctrl") {
    // Create the service for changing the control mode
    cmdClient_ = create_client<px4_msgs::srv::VehicleCommand>("/fmu/vehicle_command");
    // Publish a trigger to enable offboard control mode
    offboardCtrlModePub_ = create_publisher<px4_msgs::msg::OffboardControlMode>("fmu/in/offboard_control_mode", 10);

    // Before proceeding wait until the service is available
    while (!cmdClient_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            arm();
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
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready) {
        // auto reply = future.get()->reply;
        // uint8_t service_result_ = reply.result;
        action_done_.store(true);
    } 
    else {
        RCLCPP_INFO(this->get_logger(), "Service In-Progress...");  
    }
}

void FCUControl::publishOffBoardCtrlMode() {
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

void FCUControl::arm() {
  RCLCPP_INFO(this->get_logger(), "Arming...");	
  sendCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, VehicleCommand::ARMING_ACTION_ARM, 0.0);
}

void FCUControl::disarm() {
  RCLCPP_INFO(this->get_logger(), "Disarming");
  sendCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, VehicleCommand::ARMING_ACTION_DISARM, 0.0);
}

void FCUControl::sendCommand(uint16_t command, float param1, float param2) {
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

	auto result = cmdClient_->async_send_request(request, std::bind(&FCUControl::srvCallback, 
        this, std::placeholders::_1));
}


// Programm execution
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FCUControl>();
  rclcpp::shutdown();
  return 0;
}