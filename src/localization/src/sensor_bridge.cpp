/**
 * @file sensor_bridge.cpp
 * @brief Bridge node publishing camera and depth sensor data from Gazebo to ROS 2.
 * 
 * This node subscribes to Gazebo camera/depth topics and republishes them
 * as standard ROS 2 sensor messages (sensor_msgs/Image for camera/depth).
 * 
 * Topics published (M3):
 * - /camera/image_raw: RGB camera images
 * - /depth/image: Depth map from depth camera
 * - /camera/camera_info: Camera calibration (optional, placeholder)
 * - /imu/data: IMU data (optional, can come directly from Gazebo)
 * 
 * This implementation uses gazebo_ros bridge plugins or direct subscriptions
 * to Gazebo topics, depending on what is available in the installation.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3.hpp>

class SensorBridge : public rclcpp::Node
{
public:
  SensorBridge()
    : Node("sensor_bridge")
  {
    // Parameters
    declare_parameter<std::string>("camera_topic_in", "/camera/image_raw");
    declare_parameter<std::string>("depth_topic_in", "/depth/image");
    declare_parameter<std::string>("imu_topic_in", "/imu/data");
    declare_parameter<std::string>("camera_topic_out", "/camera/image_raw");
    declare_parameter<std::string>("depth_topic_out", "/depth/image");
    declare_parameter<std::string>("imu_topic_out", "/imu/data");
    declare_parameter<bool>("republish_camera", true);
    declare_parameter<bool>("republish_depth", true);
    declare_parameter<bool>("republish_imu", false);  // IMU usually direct from Gazebo
    declare_parameter<bool>("verbose", false);

    camera_topic_in_ = get_parameter("camera_topic_in").as_string();
    depth_topic_in_ = get_parameter("depth_topic_in").as_string();
    imu_topic_in_ = get_parameter("imu_topic_in").as_string();
    camera_topic_out_ = get_parameter("camera_topic_out").as_string();
    depth_topic_out_ = get_parameter("depth_topic_out").as_string();
    imu_topic_out_ = get_parameter("imu_topic_out").as_string();
    republish_camera_ = get_parameter("republish_camera").as_bool();
    republish_depth_ = get_parameter("republish_depth").as_bool();
    republish_imu_ = get_parameter("republish_imu").as_bool();
    verbose_ = get_parameter("verbose").as_bool();

    // Create publishers
    if (republish_camera_) {
      camera_pub_ = create_publisher<sensor_msgs::msg::Image>(camera_topic_out_, 10);
      RCLCPP_INFO(get_logger(), "Camera publisher created on: %s", camera_topic_out_.c_str());
    }

    if (republish_depth_) {
      depth_pub_ = create_publisher<sensor_msgs::msg::Image>(depth_topic_out_, 10);
      RCLCPP_INFO(get_logger(), "Depth publisher created on: %s", depth_topic_out_.c_str());
    }

    if (republish_imu_) {
      imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_out_, 10);
      RCLCPP_INFO(get_logger(), "IMU publisher created on: %s", imu_topic_out_.c_str());
    }

    // Create subscriptions
    if (republish_camera_) {
      camera_sub_ = create_subscription<sensor_msgs::msg::Image>(
        camera_topic_in_, rclcpp::QoS(5),
        [this](const sensor_msgs::msg::Image::SharedPtr msg) { camera_callback(msg); }
      );
    }

    if (republish_depth_) {
      depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        depth_topic_in_, rclcpp::QoS(5),
        [this](const sensor_msgs::msg::Image::SharedPtr msg) { depth_callback(msg); }
      );
    }

    if (republish_imu_) {
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_in_, rclcpp::QoS(10),
        [this](const sensor_msgs::msg::Imu::SharedPtr msg) { imu_callback(msg); }
      );
    }

    RCLCPP_INFO(get_logger(), "Sensor bridge initialized");
  }

private:
  void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (camera_pub_) {
      camera_pub_->publish(*msg);
      if (verbose_) {
        RCLCPP_DEBUG(get_logger(), "Camera frame: %s, %ux%u, encoding: %s",
          msg->header.frame_id.c_str(), msg->width, msg->height, msg->encoding.c_str());
      }
    }
  }

  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (depth_pub_) {
      depth_pub_->publish(*msg);
      if (verbose_) {
        RCLCPP_DEBUG(get_logger(), "Depth frame: %s, %ux%u, encoding: %s",
          msg->header.frame_id.c_str(), msg->width, msg->height, msg->encoding.c_str());
      }
    }
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (imu_pub_) {
      imu_pub_->publish(*msg);
      if (verbose_) {
        RCLCPP_DEBUG(get_logger(), "IMU frame: %s, accel: [%.3f, %.3f, %.3f]",
          msg->header.frame_id.c_str(),
          msg->linear_acceleration.x,
          msg->linear_acceleration.y,
          msg->linear_acceleration.z);
      }
    }
  }

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  // Parameters
  std::string camera_topic_in_, camera_topic_out_;
  std::string depth_topic_in_, depth_topic_out_;
  std::string imu_topic_in_, imu_topic_out_;
  bool republish_camera_, republish_depth_, republish_imu_;
  bool verbose_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SensorBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
