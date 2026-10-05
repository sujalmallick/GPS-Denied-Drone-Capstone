/**
 * @file occupancy_map.cpp
 * @brief Occupancy grid implementation.
 */

#include "mapping/occupancy_map.hpp"
#include <algorithm>
#include <cmath>

namespace mapping {

OccupancyMap::OccupancyMap(float width_m, float height_m, float resolution_m)
  : resolution_(resolution_m)
{
  width_ = static_cast<uint32_t>(width_m / resolution_m);
  height_ = static_cast<uint32_t>(height_m / resolution_m);
  origin_x_ = -width_m / 2.0f;
  origin_y_ = -height_m / 2.0f;

  data_.resize(width_ * height_, -1);  // Initialize as unknown
}

int8_t OccupancyMap::getCell(float x, float y) const
{
  uint32_t gx, gy;
  if (mapToGrid(x, y, gx, gy)) {
    return getCellByGrid(gx, gy);
  }
  return -1;  // Out of bounds
}

int8_t OccupancyMap::getCellByGrid(uint32_t gx, uint32_t gy) const
{
  if (!isValidGrid(gx, gy)) {
    return -1;
  }
  return data_[gy * width_ + gx];
}

void OccupancyMap::setCellByGrid(uint32_t gx, uint32_t gy, int8_t value)
{
  if (isValidGrid(gx, gy)) {
    data_[gy * width_ + gx] = value;
  }
}

bool OccupancyMap::isValidGrid(int32_t gx, int32_t gy) const
{
  return gx >= 0 && gx < static_cast<int32_t>(width_) &&
         gy >= 0 && gy < static_cast<int32_t>(height_);
}

bool OccupancyMap::mapToGrid(float x, float y, uint32_t& gx, uint32_t& gy) const
{
  // Map frame to grid indices
  float grid_x = (x - origin_x_) / resolution_;
  float grid_y = (y - origin_y_) / resolution_;

  if (grid_x < 0 || grid_x >= width_ || grid_y < 0 || grid_y >= height_) {
    return false;
  }

  gx = static_cast<uint32_t>(grid_x);
  gy = static_cast<uint32_t>(grid_y);
  return true;
}

void OccupancyMap::gridToMap(uint32_t gx, uint32_t gy, float& x, float& y) const
{
  x = origin_x_ + (gx + 0.5f) * resolution_;
  y = origin_y_ + (gy + 0.5f) * resolution_;
}

void OccupancyMap::clear()
{
  std::fill(data_.begin(), data_.end(), -1);
}

void OccupancyMap::getStatistics(uint32_t& free_count, uint32_t& occupied_count, uint32_t& unknown_count) const
{
  free_count = 0;
  occupied_count = 0;
  unknown_count = 0;

  for (int8_t cell : data_) {
    if (cell == -1) {
      unknown_count++;
    } else if (cell == 0) {
      free_count++;
    } else if (cell == 100) {
      occupied_count++;
    }
  }
}

void OccupancyMap::raycast(uint32_t sx, uint32_t sy, uint32_t ex, uint32_t ey)
{
  // Bresenham line algorithm
  int dx = static_cast<int>(ex) - static_cast<int>(sx);
  int dy = static_cast<int>(ey) - static_cast<int>(sy);

  int steps = std::abs(dx) > std::abs(dy) ? std::abs(dx) : std::abs(dy);
  if (steps == 0) steps = 1;

  float step_x = static_cast<float>(dx) / steps;
  float step_y = static_cast<float>(dy) / steps;

  for (int i = 0; i < steps - 1; ++i) {  // Don't mark the endpoint
    float x = sx + step_x * i;
    float y = sy + step_y * i;

    uint32_t gx = static_cast<uint32_t>(x);
    uint32_t gy = static_cast<uint32_t>(y);

    if (isValidGrid(gx, gy)) {
      // Only update if currently unknown
      if (data_[gy * width_ + gx] == -1) {
        data_[gy * width_ + gx] = 0;  // FREE
      }
    }
  }
}

void OccupancyMap::updateFromDepth(
  const geometry_msgs::msg::Pose& sensor_pose,
  const std::vector<float>& depth_image,
  uint32_t img_width, uint32_t img_height,
  float fx, float fy, float cx, float cy,
  float max_range)
{
  // Get sensor position and orientation
  float sx = sensor_pose.position.x;
  float sy = sensor_pose.position.y;
  float sz = sensor_pose.position.z;

  // Convert sensor pose to grid
  uint32_t sgx, sgy;
  if (!mapToGrid(sx, sy, sgx, sgy)) {
    return;  // Sensor outside map
  }

  // For each pixel in depth image
  for (uint32_t v = 0; v < img_height; ++v) {
    for (uint32_t u = 0; u < img_width; ++u) {
      float d = depth_image[v * img_width + u];

      // Skip invalid or too-far depth
      if (std::isnan(d) || std::isinf(d) || d <= 0.0f || d > max_range) {
        continue;
      }

      // Backproject to camera frame
      float x_cam = (u - cx) * d / fx;
      float y_cam = (v - cy) * d / fy;
      float z_cam = d;

      // TODO: Apply camera-to-body transform, then body-to-map transform
      // For now, assume camera is forward-facing on the drone
      // This is a simplification; real implementation needs full transform chain

      float x_map = sx + x_cam;  // Simplified
      float y_map = sy + y_cam;

      // Clamp to max range to avoid out-of-bounds raycasts
      if (std::sqrt((x_map - sx) * (x_map - sx) + (y_map - sy) * (y_map - sy)) > max_range) {
        continue;
      }

      uint32_t egx, egy;
      if (mapToGrid(x_map, y_map, egx, egy)) {
        // Mark occupied
        setCellByGrid(egx, egy, 100);

        // Raycast from sensor to hit point to mark free cells
        raycast(sgx, sgy, egx, egy);
      }
    }
  }
}

}  // namespace mapping
