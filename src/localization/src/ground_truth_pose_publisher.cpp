/**
 * @file ground_truth_pose_publisher.cpp
 * @brief Ground-truth pose publisher for M3-M10 using Gazebo odometry data.
 * 
 * This node reads the drone's ground-truth pose from Gazebo and publishes it
 * as a ROS 2 PoseStamped message on the "ground_truth_pose" topic.
 * 
 * This is development-mode localization (M3-M10). For production (M11),
 * this node will be replaced by a VSLAM/VIO node on the same topic interface,
 * requiring no changes to mapping or exploration code.
 * 
 * Frame conventions (REP-105):
 * - map: Fixed world frame (origin at Gazebo world origin)
 * - odom: Odometry frame (typically same as map for ground truth)
 * - base_link: Drone body frame
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class GroundTruthPosePublisher : public rclcpp::Node
{
public:
  GroundTruthPosePublisher()
    : Node("ground_truth_pose_publisher"),
      last_pose_time_(0)
  {
    // Parameters
    declare_parameter<std::string>("gazebo_odometry_topic", "/odometry/gazebo");
    declare_parameter<std::string>("ros_pose_topic", "ground_truth_pose");
    declare_parameter<bool>("publish_tf", true);
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("base_link_frame", "base_link");
    declare_parameter<bool>("verbose", false);

    gazebo_odom_topic_ = get_parameter("gazebo_odometry_topic").as_string();
    ros_pose_topic_ = get_parameter("ros_pose_topic").as_string();
    publish_tf_ = get_parameter("publish_tf").as_bool();
    map_frame_ = get_parameter("map_frame").as_string();
    base_link_frame_ = get_parameter("base_link_frame").as_string();
    verbose_ = get_parameter("verbose").as_bool();

    // Create publisher
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(ros_pose_topic_, 10);

    // Create TF broadcaster if enabled
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    }

    // Subscribe to Gazebo odometry
    // Note: Gazebo publishes ground-truth pose via /odom or model state topics
    // For development, we use a simple timer-based approach reading Gazebo state
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      gazebo_odom_topic_,
      rclcpp::QoS(10),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { odom_callback(msg); }
    );

    if (verbose_) {
      RCLCPP_INFO(get_logger(),
        "Ground-truth pose publisher initialized on topic: %s", ros_pose_topic_.c_str());
      RCLCPP_INFO(get_logger(), "Map frame: %s, Base link frame: %s",
        map_frame_.c_str(), base_link_frame_.c_str());
    }
  }

private:
  /// Callback for Gazebo odometry data
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Create pose message
    auto pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
    pose_msg->header.stamp = msg->header.stamp;
    pose_msg->header.frame_id = map_frame_;
    pose_msg->pose = msg->pose.pose;

    // Publish pose
    pose_pub_->publish(*pose_msg);

    // Publish TF if enabled
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = msg->header.stamp;
      transform.header.frame_id = map_frame_;
      transform.child_frame_id = base_link_frame_;
      transform.transform.translation.x = msg->pose.pose.position.x;
      transform.transform.translation.y = msg->pose.pose.position.y;
      transform.transform.translation.z = msg->pose.pose.position.z;
      transform.transform.rotation = msg->pose.pose.orientation;

      tf_broadcaster_->sendTransform(transform);
    }

    // Verbose logging
    if (verbose_ && (msg->header.stamp.sec - last_pose_time_ > 5)) {
      RCLCPP_INFO(get_logger(),
        "Pose: [%.2f, %.2f, %.2f] (map frame)",
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z
      );
      last_pose_time_ = msg->header.stamp.sec;
    }
  }

  // Publishers and subscribers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Parameters
  std::string gazebo_odom_topic_;
  std::string ros_pose_topic_;
  std::string map_frame_;
  std::string base_link_frame_;
  bool publish_tf_;
  bool verbose_;
  uint32_t last_pose_time_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GroundTruthPosePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
