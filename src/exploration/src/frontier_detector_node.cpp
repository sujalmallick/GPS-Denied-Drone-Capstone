/**
 * @file frontier_detector_node.cpp
 * @brief Main node orchestrating frontier detection, clustering, and selection (M5-M7).
 * 
 * Subscribes to:
 * - /map (nav_msgs/OccupancyGrid): Occupancy grid from mapping node
 * - ground_truth_pose (geometry_msgs/PoseStamped): Current drone pose
 * 
 * Publishes:
 * - /exploration/frontiers (visualization_msgs/MarkerArray): Frontier cells
 * - /exploration/clusters (visualization_msgs/MarkerArray): Frontier clusters
 * - /exploration/selected_frontier (geometry_msgs/PointStamped): Selected cluster centroid
 * - /exploration/frontier_info (std_msgs/String): Info message with selection details
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <sstream>

#include "exploration/frontier_detector.hpp"

using namespace exploration;

class FrontierDetectorNode : public rclcpp::Node
{
public:
  FrontierDetectorNode()
    : Node("frontier_detector_node"),
      map_width_(0), map_height_(0), map_resolution_(0.0f),
      current_pose_valid_(false)
  {
    // Declare parameters
    declare_parameter<float>("distance_cost_weight", 0.1f);
    declare_parameter<uint32_t>("min_cluster_size", 1);
    declare_parameter<bool>("verbose", false);
    declare_parameter<bool>("publish_visualization", true);

    distance_cost_weight_ = get_parameter("distance_cost_weight").as_double();
    min_cluster_size_ = get_parameter("min_cluster_size").as_int();
    verbose_ = get_parameter("verbose").as_bool();
    publish_visualization_ = get_parameter("publish_visualization").as_bool();

    // Create publishers
    selected_frontier_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "/exploration/selected_frontier", 10);

    info_pub_ = create_publisher<std_msgs::msg::String>(
      "/exploration/frontier_info", 10);

    if (publish_visualization_) {
      frontier_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/exploration/frontiers", 10);

      cluster_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/exploration/clusters", 10);
    }

    // Create subscribers
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map",
      rclcpp::QoS(5),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) { map_callback(msg); }
    );

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "ground_truth_pose",
      rclcpp::QoS(10),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_pose_ = msg->pose;
        current_pose_valid_ = true;
      }
    );

    RCLCPP_INFO(get_logger(), "Frontier detector node initialized");
  }

private:
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    if (!current_pose_valid_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Pose not yet received, cannot process map");
      return;
    }

    // Extract map info
    map_width_ = msg->info.width;
    map_height_ = msg->info.height;
    map_resolution_ = msg->info.resolution;
    map_origin_x_ = msg->info.origin.position.x;
    map_origin_y_ = msg->info.origin.position.y;

    // M5: Detect frontiers
    auto frontiers = FrontierDetector::detectFrontiers(msg->data, map_width_, map_height_);

    if (verbose_) {
      RCLCPP_INFO(get_logger(), "M5: Detected %zu frontier cells", frontiers.size());
    }

    if (frontiers.empty()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "No frontier cells found - exploration may be complete");
      return;
    }

    // M6: Cluster frontiers
    auto clusters = FrontierClustering::clusterFrontiers(frontiers, map_width_, map_height_);

    if (verbose_) {
      RCLCPP_INFO(get_logger(), "M6: Clustered into %zu frontier clusters", clusters.size());
    }

    // Compute centroids
    for (auto& cluster : clusters) {
      cluster.computeCentroid(map_resolution_, map_origin_x_, map_origin_y_);
    }

    // M7: Select best frontier
    auto selection = FrontierSelector::selectBestFrontier(
      clusters,
      current_pose_.position.x,
      current_pose_.position.y,
      distance_cost_weight_,
      min_cluster_size_
    );

    if (!selection.valid) {
      RCLCPP_WARN(get_logger(), "No valid frontier clusters found");
      return;
    }

    if (verbose_) {
      RCLCPP_INFO(get_logger(),
        "M7: Selected cluster %u with score %.2f",
        selection.selected_cluster_id, selection.score);
    }

    // Publish selected frontier
    const auto& selected_cluster = clusters[selection.selected_cluster_id];
    auto selected_msg = std::make_shared<geometry_msgs::msg::PointStamped>();
    selected_msg->header.stamp = msg->header.stamp;
    selected_msg->header.frame_id = "map";
    selected_msg->point.x = selected_cluster.centroid_x;
    selected_msg->point.y = selected_cluster.centroid_y;
    selected_msg->point.z = 0.0f;

    selected_frontier_pub_->publish(*selected_msg);

    // Publish info message
    std::ostringstream info_stream;
    info_stream << "M5-M7 Frontier Info:\n"
                << "  Frontiers: " << frontiers.size() << " cells\n"
                << "  Clusters: " << clusters.size() << "\n"
                << "  Selected: Cluster " << selection.selected_cluster_id
                << " (" << selected_cluster.cells.size() << " cells, score="
                << selection.score << ")\n"
                << "  Centroid: [" << selected_cluster.centroid_x << ", "
                << selected_cluster.centroid_y << "]\n"
                << "  Drone: [" << current_pose_.position.x << ", "
                << current_pose_.position.y << "]";

    auto info_msg = std::make_shared<std_msgs::msg::String>();
    info_msg->data = info_stream.str();
    info_pub_->publish(*info_msg);

    // Publish visualization markers
    if (publish_visualization_) {
      publish_frontier_markers(msg->header.stamp, frontiers);
      publish_cluster_markers(msg->header.stamp, clusters, selection.selected_cluster_id);
    }
  }

  void publish_frontier_markers(const builtin_interfaces::msg::Time& stamp,
                                 const std::vector<FrontierCell>& frontiers)
  {
    auto marker_array = std::make_shared<visualization_msgs::msg::MarkerArray>();

    // Clear previous markers
    auto clear_marker = std::make_shared<visualization_msgs::msg::Marker>();
    clear_marker->action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array->markers.push_back(*clear_marker);

    // Create frontier points marker
    auto marker = std::make_shared<visualization_msgs::msg::Marker>();
    marker->header.stamp = stamp;
    marker->header.frame_id = "map";
    marker->id = 0;
    marker->type = visualization_msgs::msg::Marker::POINTS;
    marker->action = visualization_msgs::msg::Marker::ADD;
    marker->pose.orientation.w = 1.0;
    marker->scale.x = map_resolution_ * 0.5f;
    marker->scale.y = map_resolution_ * 0.5f;
    marker->color.r = 0.0f;
    marker->color.g = 1.0f;
    marker->color.b = 1.0f;
    marker->color.a = 0.8f;

    for (const auto& cell : frontiers) {
      geometry_msgs::msg::Point p;
      p.x = map_origin_x_ + (cell.gx + 0.5f) * map_resolution_;
      p.y = map_origin_y_ + (cell.gy + 0.5f) * map_resolution_;
      p.z = 0.0f;
      marker->points.push_back(p);
    }

    marker_array->markers.push_back(*marker);
    frontier_markers_pub_->publish(*marker_array);
  }

  void publish_cluster_markers(const builtin_interfaces::msg::Time& stamp,
                               const std::vector<FrontierCluster>& clusters,
                               uint32_t selected_id)
  {
    auto marker_array = std::make_shared<visualization_msgs::msg::MarkerArray>();

    for (size_t i = 0; i < clusters.size(); ++i) {
      const auto& cluster = clusters[i];
      auto marker = std::make_shared<visualization_msgs::msg::Marker>();

      marker->header.stamp = stamp;
      marker->header.frame_id = "map";
      marker->id = static_cast<int32_t>(i);
      marker->type = visualization_msgs::msg::Marker::SPHERE;
      marker->action = visualization_msgs::msg::Marker::ADD;

      marker->pose.position.x = cluster.centroid_x;
      marker->pose.position.y = cluster.centroid_y;
      marker->pose.position.z = 0.5f;
      marker->pose.orientation.w = 1.0;

      marker->scale.x = 0.5f;
      marker->scale.y = 0.5f;
      marker->scale.z = 0.5f;

      if (cluster.cluster_id == selected_id) {
        marker->color.r = 1.0f;  // Red for selected
        marker->color.g = 0.0f;
        marker->color.b = 0.0f;
      } else {
        marker->color.r = 0.0f;  // Blue for unselected
        marker->color.g = 0.0f;
        marker->color.b = 1.0f;
      }
      marker->color.a = 0.8f;

      marker_array->markers.push_back(*marker);
    }

    cluster_markers_pub_->publish(*marker_array);
  }

  // Publishers and subscribers
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr selected_frontier_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr info_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr frontier_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_markers_pub_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  // Map info
  uint32_t map_width_, map_height_;
  float map_resolution_;
  float map_origin_x_, map_origin_y_;

  // Pose
  geometry_msgs::msg::Pose current_pose_;
  bool current_pose_valid_;

  // Parameters
  float distance_cost_weight_;
  uint32_t min_cluster_size_;
  bool verbose_;
  bool publish_visualization_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FrontierDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
