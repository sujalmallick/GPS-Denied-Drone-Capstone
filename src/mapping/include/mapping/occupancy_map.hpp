/**
 * @file occupancy_map.hpp
 * @brief Occupancy grid representation and update logic.
 */

#ifndef MAPPING__OCCUPANCY_MAP_HPP_
#define MAPPING__OCCUPANCY_MAP_HPP_

#include <vector>
#include <cstdint>
#include <geometry_msgs/msg/pose.hpp>

namespace mapping {

/**
 * @class OccupancyMap
 * @brief 2D occupancy grid representation with raycasting-based update.
 * 
 * Cell states:
 * - -1 (UNKNOWN): Not yet observed
 * -  0 (FREE): Observed as free space
 * -  100 (OCCUPIED): Observed as occupied
 * 
 * Convention: Origin at center (0, 0) maps to grid center; Z-axis ignored.
 */
class OccupancyMap
{
public:
  /**
   * @brief Initialize occupancy map.
   * @param width_m Map width in meters
   * @param height_m Map height in meters
   * @param resolution_m Cell resolution in meters (e.g., 0.1 m/cell)
   */
  OccupancyMap(float width_m, float height_m, float resolution_m);

  /**
   * @brief Update map with depth data from a sensor pose.
   * @param sensor_pose Pose of the depth camera in map frame
   * @param depth_image Depth image (single-channel, float32, meters)
   * @param fx, fy Focal lengths in pixels
   * @param cx, cy Principal point in pixels
   * @param max_range Maximum valid depth range (meters)
   */
  void updateFromDepth(
    const geometry_msgs::msg::Pose& sensor_pose,
    const std::vector<float>& depth_image,
    uint32_t img_width, uint32_t img_height,
    float fx, float fy, float cx, float cy,
    float max_range);

  /**
   * @brief Get cell occupancy value.
   * @param x X coordinate in map frame (meters)
   * @param y Y coordinate in map frame (meters)
   * @return -1 (unknown), 0 (free), 100 (occupied)
   */
  int8_t getCell(float x, float y) const;

  /**
   * @brief Get cell by grid indices.
   * @param gx Grid X index
   * @param gy Grid Y index
   * @return -1 (unknown), 0 (free), 100 (occupied)
   */
  int8_t getCellByGrid(uint32_t gx, uint32_t gy) const;

  /**
   * @brief Set cell occupancy value.
   * @param gx Grid X index
   * @param gy Grid Y index
   * @param value Occupancy value (-1, 0, or 100)
   */
  void setCellByGrid(uint32_t gx, uint32_t gy, int8_t value);

  /**
   * @brief Get map data as flattened array (row-major, origin at bottom-left per ROS convention).
   * @return Vector of occupancy values
   */
  const std::vector<int8_t>& getData() const { return data_; }

  /**
   * @brief Get mutable map data.
   */
  std::vector<int8_t>& getData() { return data_; }

  // Accessors
  uint32_t getWidth() const { return width_; }
  uint32_t getHeight() const { return height_; }
  float getResolution() const { return resolution_; }
  float getOriginX() const { return origin_x_; }
  float getOriginY() const { return origin_y_; }

  /**
   * @brief Check if grid coordinates are valid.
   */
  bool isValidGrid(int32_t gx, int32_t gy) const;

  /**
   * @brief Convert map frame coordinates to grid indices.
   */
  bool mapToGrid(float x, float y, uint32_t& gx, uint32_t& gy) const;

  /**
   * @brief Convert grid indices to map frame coordinates (cell center).
   */
  void gridToMap(uint32_t gx, uint32_t gy, float& x, float& y) const;

  /**
   * @brief Clear map (set all cells to UNKNOWN).
   */
  void clear();

  /**
   * @brief Get occupancy statistics.
   * @param free_count Output: count of FREE cells
   * @param occupied_count Output: count of OCCUPIED cells
   * @param unknown_count Output: count of UNKNOWN cells
   */
  void getStatistics(uint32_t& free_count, uint32_t& occupied_count, uint32_t& unknown_count) const;

private:
  /// Bresenham line raycasting for ray from sensor_pos to hit_pos.
  /// Marks cells as FREE along the ray except the final cell.
  void raycast(uint32_t sx, uint32_t sy, uint32_t ex, uint32_t ey);

  uint32_t width_;      ///< Width in cells
  uint32_t height_;     ///< Height in cells
  float resolution_;    ///< Meters per cell
  float origin_x_;      ///< X origin in map frame
  float origin_y_;      ///< Y origin in map frame

  std::vector<int8_t> data_;  ///< Flattened grid (row-major)
};

}  // namespace mapping

#endif  // MAPPING__OCCUPANCY_MAP_HPP_
