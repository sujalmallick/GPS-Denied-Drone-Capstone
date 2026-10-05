/*
 * M2: Minimal PX4 Control Node
 * 
 * This node demonstrates basic ROS 2 <-> PX4 SITL communication via uXRCE-DDS.
 * 
 * Functionality:
 * - Subscribes to PX4 vehicle status and publishes telemetry
 * - Provides a service to arm/disarm the drone
 * - Monitors vehicle state and outputs status messages
 * 
 * Dependencies:
 * - px4_msgs (PX4 ROS 2 message definitions)
 * - uXRCE-DDS bridge running between PX4 SITL and ROS 2
 * 
 * Note: This is a minimal M2 implementation. Sensor processing and 
 * exploration logic are deferred to M3+.
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>

#include <iostream>
#include <chrono>
#include <memory>

class PX4ControlNode : public rclcpp::Node
{
public:
  PX4ControlNode()
    : Node("px4_control_node"),
      vehicle_armed_(false),
      last_status_time_(0)
  {
    // Declare and retrieve parameters
    this->declare_parameter<int>("domain_id", 0);
    this->declare_parameter<bool>("verbose", false);

    domain_id_ = this->get_parameter("domain_id").as_int();
    verbose_ = this->get_parameter("verbose").as_bool();

    if (verbose_) {
      RCLCPP_INFO(this->get_logger(), "PX4 Control Node starting (domain_id=%d)", domain_id_);
    }

    // Subscription: Vehicle Status
    vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
      "fmu/vehicle_status/out",
      rclcpp::QoS(10),
      std::bind(&PX4ControlNode::vehicle_status_callback, this, std::placeholders::_1)
    );

    // Subscription: Vehicle Control Mode
    vehicle_control_mode_sub_ = this->create_subscription<px4_msgs::msg::VehicleControlMode>(
      "fmu/vehicle_control_mode/out",
      rclcpp::QoS(10),
      std::bind(&PX4ControlNode::vehicle_control_mode_callback, this, std::placeholders::_1)
    );

    // Subscription: Sensor Combined (for IMU data)
    sensor_combined_sub_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
      "fmu/sensor_combined/out",
      rclcpp::QoS(50),
      std::bind(&PX4ControlNode::sensor_combined_callback, this, std::placeholders::_1)
    );

    // Publisher: Vehicle Command (to send commands to PX4)
    vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
      "fmu/vehicle_command/in",
      rclcpp::QoS(10)
    );

    // Publisher: Telemetry status (for monitoring)
    telemetry_pub_ = this->create_publisher<std_msgs::msg::String>(
      "control/telemetry",
      rclcpp::QoS(10)
    );

    // Service: Arm/Disarm drone
    arm_service_ = this->create_service<std_srvs::srv::SetBool>(
      "control/arm",
      std::bind(&PX4ControlNode::handle_arm_service, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Timer for periodic status publishing
    status_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&PX4ControlNode::publish_status, this)
    );

    RCLCPP_INFO(this->get_logger(), "PX4 Control Node initialized successfully");
  }

private:
  // Callbacks
  void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
  {
    if (verbose_) {
      RCLCPP_DEBUG(this->get_logger(), 
        "Vehicle Status: armed=%d, mode=%d, frame=%d",
        msg->armed, msg->vehicle_type, msg->vehicle_frame);
    }
    
    vehicle_armed_ = (msg->armed == px4_msgs::msg::VehicleStatus::VEHICLE_STATUS_ARMED);
    last_status_time_ = this->now().nanoseconds();
  }

  void vehicle_control_mode_callback(const px4_msgs::msg::VehicleControlMode::SharedPtr msg)
  {
    if (verbose_) {
      RCLCPP_DEBUG(this->get_logger(), 
        "Vehicle Control Mode: manual=%d, stabilized=%d, offboard=%d",
        msg->flag_control_manual_enabled, 
        msg->flag_control_stabilized_enabled,
        msg->flag_control_offboard_enabled);
    }
  }

  void sensor_combined_callback(const px4_msgs::msg::SensorCombined::SharedPtr msg)
  {
    if (verbose_) {
      RCLCPP_DEBUG(this->get_logger(), 
        "Sensor Data: accel=[%.2f, %.2f, %.2f]",
        msg->accelerometer_m_s2[0], 
        msg->accelerometer_m_s2[1],
        msg->accelerometer_m_s2[2]);
    }
  }

  void handle_arm_service(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    // Create vehicle command message
    auto cmd = px4_msgs::msg::VehicleCommand{};
    cmd.timestamp = this->now().nanoseconds() / 1000;  // Convert to microseconds
    
    if (request->data) {
      // ARM command
      cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
      cmd.param1 = 1.0f;  // ARM
      
      RCLCPP_INFO(this->get_logger(), "Sending ARM command to PX4");
    } else {
      // DISARM command
      cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
      cmd.param1 = 0.0f;  // DISARM
      
      RCLCPP_INFO(this->get_logger(), "Sending DISARM command to PX4");
    }

    cmd.param2 = 0.0f;  // Force disarm (set to 21196.0f for force)
    cmd.param3 = 0.0f;
    cmd.param4 = 0.0f;
    cmd.param5 = 0.0f;
    cmd.param6 = 0.0f;
    cmd.param7 = 0.0f;
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.from_external = true;

    // Publish command
    vehicle_command_pub_->publish(cmd);

    // Prepare response
    response->success = true;
    response->message = (request->data) ? "ARM command sent" : "DISARM command sent";
  }

  void publish_status()
  {
    // Publish current status as a string for easy monitoring
    auto msg = std_msgs::msg::String();
    
    int64_t time_since_status = this->now().nanoseconds() - last_status_time_;
    bool status_fresh = (time_since_status < 2000000000);  // 2 seconds in nanoseconds

    msg.data = std::string("Vehicle Status: ") +
               (vehicle_armed_ ? "ARMED" : "DISARMED") +
               std::string(" | Status Fresh: ") +
               (status_fresh ? "YES" : "NO") +
               std::string(" | Time Since Update: ") +
               std::to_string(time_since_status / 1000000) + std::string("ms");

    telemetry_pub_->publish(msg);

    if (verbose_) {
      RCLCPP_DEBUG(this->get_logger(), "%s", msg.data.c_str());
    }
  }

  // Subscriptions
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr vehicle_control_mode_sub_;
  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sensor_combined_sub_;

  // Publishers
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr telemetry_pub_;

  // Services
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_service_;

  // Timer
  rclcpp::TimerBase::SharedPtr status_timer_;

  // Node state
  bool vehicle_armed_;
  int64_t last_status_time_;
  int domain_id_;
  bool verbose_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PX4ControlNode>();
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}
