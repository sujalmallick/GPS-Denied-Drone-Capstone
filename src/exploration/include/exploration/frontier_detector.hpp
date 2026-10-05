/**
 * @file frontier_detector.hpp
 * @brief Frontier detection, clustering, and selection for M5-M7.
 */

#ifndef EXPLORATION__FRONTIER_DETECTOR_HPP_
#define EXPLORATION__FRONTIER_DETECTOR_HPP_

#include <vector>
#include <cstdint>
#include <geometry_msgs/msg/point.hpp>

namespace exploration {

/// Frontier cell (map grid position)
struct FrontierCell
{
  uint32_t gx;  ///< Grid X
  uint32_t gy;  ///< Grid Y

  FrontierCell(uint32_t x = 0, uint32_t y = 0) : gx(x), gy(y) {}

  bool operator==(const FrontierCell& other) const {
    return gx == other.gx && gy == other.gy;
  }
};

/// Frontier cluster (group of connected frontier cells)
struct FrontierCluster
{
  uint32_t cluster_id;
  std::vector<FrontierCell> cells;
  float centroid_x;  ///< Centroid in map frame (meters)
  float centroid_y;

  FrontierCluster(uint32_t id = 0)
    : cluster_id(id), centroid_x(0.0f), centroid_y(0.0f) {}

  /// Compute centroid from cells (requires map resolution knowledge)
  void computeCentroid(float resolution, float origin_x, float origin_y);

  /// Information gain (approximately: number of UNKNOWN cells adjacent to cluster)
  uint32_t getInformationGain() const { return cells.size(); }
};

/**
 * @class FrontierDetector
 * @brief Detect frontier cells from occupancy grid (M5).
 * 
 * Frontier = FREE cell adjacent to at least one UNKNOWN cell.
 * Uses 8-connectivity.
 */
class FrontierDetector
{
public:
  /**
   * @brief Detect frontiers in occupancy grid.
   * @param grid Flattened occupancy grid (row-major)
   * @param width Grid width in cells
   * @param height Grid height in cells
   * @return Vector of frontier cells
   */
  static std::vector<FrontierCell> detectFrontiers(
    const std::vector<int8_t>& grid,
    uint32_t width, uint32_t height);

private:
  /// Check if cell is FREE and has an UNKNOWN neighbor
  static bool isFrontier(
    const std::vector<int8_t>& grid,
    uint32_t width, uint32_t height,
    uint32_t gx, uint32_t gy);
};

/**
 * @class FrontierClustering
 * @brief Cluster frontier cells using connected-component labeling (M6).
 * 
 * Groups spatially connected frontier cells into discrete clusters.
 * Uses 8-connectivity.
 */
class FrontierClustering
{
public:
  /**
   * @brief Cluster frontier cells.
   * @param frontier_cells Vector of frontier cells to cluster
   * @param width Grid width
   * @param height Grid height
   * @return Vector of clusters
   */
  static std::vector<FrontierCluster> clusterFrontiers(
    const std::vector<FrontierCell>& frontier_cells,
    uint32_t width, uint32_t height);

private:
  /// BFS to find all connected cells in a cluster
  static void bfs(
    uint32_t start_gx, uint32_t start_gy,
    std::vector<std::vector<int>>& visited,
    const std::vector<FrontierCell>& frontier_cells,
    uint32_t width, uint32_t height,
    FrontierCluster& cluster,
    const std::vector<bool>& is_frontier_map);
};

/**
 * @class FrontierSelector
 * @brief Select best frontier cluster using scoring function (M7).
 * 
 * Scoring: score = information_gain - distance_cost
 * Where:
 * - information_gain = cluster size (number of frontier cells)
 * - distance_cost = distance from drone to cluster centroid * cost_weight
 */
class FrontierSelector
{
public:
  struct SelectionResult
  {
    uint32_t selected_cluster_id;
    float score;
    bool valid;  ///< False if no valid clusters found

    SelectionResult(uint32_t id = 0, float s = 0.0f, bool v = false)
      : selected_cluster_id(id), score(s), valid(v) {}
  };

  /**
   * @brief Select best frontier cluster.
   * @param clusters Frontier clusters to choose from
   * @param drone_x Drone X position in map frame
   * @param drone_y Drone Y position in map frame
   * @param distance_cost_weight Weight for distance penalty (default 0.1)
   * @param min_cluster_size Minimum cluster size to consider (default 1)
   * @return Selection result with best cluster ID and score
   */
  static SelectionResult selectBestFrontier(
    const std::vector<FrontierCluster>& clusters,
    float drone_x, float drone_y,
    float distance_cost_weight = 0.1f,
    uint32_t min_cluster_size = 1);

private:
  /// Compute distance from drone to cluster centroid
  static float computeDistance(
    float drone_x, float drone_y,
    float cluster_cx, float cluster_cy);
};

}  // namespace exploration

#endif  // EXPLORATION__FRONTIER_DETECTOR_HPP_
