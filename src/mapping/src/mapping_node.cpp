/**
 * @file mapping_node.cpp
 * @brief Occupancy grid mapping node (M4).
 * 
 * Subscribes to:
 * - ground_truth_pose (geometry_msgs/PoseStamped): Current drone pose
 * - /depth/image (sensor_msgs/Image): Depth data from drone camera
 * 
 * Publishes:
 * - /map (nav_msgs/OccupancyGrid): Updated occupancy grid
 * 
 * Configuration:
 * - map_width_m: Map extent in X (default 40m)
 * - map_height_m: Map extent in Y (default 40m)
 * - map_resolution_m: Grid resolution (default 0.1m per cell)
 * - camera_fx, fy, cx, cy: Camera intrinsics (pinhole model)
 * - depth_max_range: Max valid depth (default 10m)
 * - publish_rate_hz: Map publication rate (default 2 Hz)
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <memory>
#include <chrono>

#include "mapping/occupancy_map.hpp"

class MappingNode : public rclcpp::Node
{
public:
  MappingNode()
    : Node("mapping_node"),
      current_pose_valid_(false),
      last_update_time_(0)
  {
    // Declare parameters
    declare_parameter<float>("map_width_m", 40.0f);
    declare_parameter<float>("map_height_m", 40.0f);
    declare_parameter<float>("map_resolution_m", 0.1f);
    declare_parameter<float>("camera_fx", 518.0f);  // Typical pinhole model
    declare_parameter<float>("camera_fy", 518.0f);
    declare_parameter<float>("camera_cx", 320.0f);
    declare_parameter<float>("camera_cy", 240.0f);
    declare_parameter<float>("depth_max_range", 10.0f);
    declare_parameter<int>("publish_rate_hz", 2);
    declare_parameter<bool>("verbose", false);

    // Get parameters
    float map_width = get_parameter("map_width_m").as_double();
    float map_height = get_parameter("map_height_m").as_double();
    float map_resolution = get_parameter("map_resolution_m").as_double();
    camera_fx_ = get_parameter("camera_fx").as_double();
    camera_fy_ = get_parameter("camera_fy").as_double();
    camera_cx_ = get_parameter("camera_cx").as_double();
    camera_cy_ = get_parameter("camera_cy").as_double();
    depth_max_range_ = get_parameter("depth_max_range").as_double();
    int publish_rate = get_parameter("publish_rate_hz").as_int();
    verbose_ = get_parameter("verbose").as_bool();

    // Create occupancy map
    map_ = std::make_shared<mapping::OccupancyMap>(map_width, map_height, map_resolution);

    // Create publisher and subscribers
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "ground_truth_pose",
      rclcpp::QoS(10),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_pose_ = msg->pose;
        current_pose_valid_ = true;
      }
    );

    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/depth/image",
      rclcpp::QoS(5),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) { depth_callback(msg); }
    );

    // Create timer for periodic publication
    auto timer_period = std::chrono::milliseconds(1000 / publish_rate);
    timer_ = create_wall_timer(
      timer_period,
      [this]() { publish_map(); }
    );

    RCLCPP_INFO(get_logger(),
      "Mapping node initialized: %.1fx%.1fm @ %.2fm/cell",
      map_width, map_height, map_resolution);
  }

private:
  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (!current_pose_valid_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Pose not yet received, cannot update map");
      return;
    }

    // Convert depth image
    cv::Mat depth_cv;
    try {
      auto cv_img = cv_bridge::toCvShare(msg, "");
      depth_cv = cv_img->image;
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // Convert to float and reshape to vector
    if (depth_cv.type() != CV_32F) {
      depth_cv.convertTo(depth_cv, CV_32F);
    }

    std::vector<float> depth_vec(depth_cv.begin<float>(), depth_cv.end<float>());

    // Update map
    map_->updateFromDepth(
      current_pose_,
      depth_vec,
      msg->width, msg->height,
      camera_fx_, camera_fy_,
      camera_cx_, camera_cy_,
      depth_max_range_
    );

    if (verbose_) {
      uint32_t free_count, occupied_count, unknown_count;
      map_->getStatistics(free_count, occupied_count, unknown_count);

      last_update_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
      ).count();

      RCLCPP_DEBUG(get_logger(),
        "Map updated: free=%u, occupied=%u, unknown=%u at pose [%.2f, %.2f, %.2f]",
        free_count, occupied_count, unknown_count,
        current_pose_.position.x,
        current_pose_.position.y,
        current_pose_.position.z);
    }
  }

  void publish_map()
  {
    auto msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();

    msg->header.frame_id = "map";
    msg->header.stamp = now();

    msg->info.resolution = map_->getResolution();
    msg->info.width = map_->getWidth();
    msg->info.height = map_->getHeight();

    // ROS convention: origin at bottom-left
    msg->info.origin.position.x = map_->getOriginX();
    msg->info.origin.position.y = map_->getOriginY();
    msg->info.origin.position.z = 0.0f;
    msg->info.origin.orientation.w = 1.0f;

    // Copy data
    msg->data = map_->getData();

    // Publish
    map_pub_->publish(*msg);

    if (verbose_) {
      RCLCPP_DEBUG(get_logger(), "Map published: %u cells", msg->data.size());
    }
  }

  std::shared_ptr<mapping::OccupancyMap> map_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Pose current_pose_;
  bool current_pose_valid_;

  float camera_fx_, camera_fy_, camera_cx_, camera_cy_;
  float depth_max_range_;
  bool verbose_;
  int64_t last_update_time_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MappingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
