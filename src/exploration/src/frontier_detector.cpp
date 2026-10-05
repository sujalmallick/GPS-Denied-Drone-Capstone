/**
 * @file frontier_detector.cpp
 * @brief Frontier detection implementation (M5).
 */

#include "exploration/frontier_detector.hpp"
#include <algorithm>
#include <cmath>
#include <queue>

namespace exploration {

std::vector<FrontierCell> FrontierDetector::detectFrontiers(
  const std::vector<int8_t>& grid,
  uint32_t width, uint32_t height)
{
  std::vector<FrontierCell> frontiers;

  for (uint32_t gy = 0; gy < height; ++gy) {
    for (uint32_t gx = 0; gx < width; ++gx) {
      if (isFrontier(grid, width, height, gx, gy)) {
        frontiers.emplace_back(gx, gy);
      }
    }
  }

  return frontiers;
}

bool FrontierDetector::isFrontier(
  const std::vector<int8_t>& grid,
  uint32_t width, uint32_t height,
  uint32_t gx, uint32_t gy)
{
  // Cell must be FREE
  int8_t cell = grid[gy * width + gx];
  if (cell != 0) {
    return false;  // Not free
  }

  // Check 8-connected neighbors for at least one UNKNOWN cell
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      if (dx == 0 && dy == 0) continue;  // Skip center

      int ngx = static_cast<int>(gx) + dx;
      int ngy = static_cast<int>(gy) + dy;

      // Check bounds
      if (ngx < 0 || ngx >= static_cast<int>(width) ||
          ngy < 0 || ngy >= static_cast<int>(height)) {
        // Boundary of map is treated as unknown
        return true;
      }

      int8_t neighbor = grid[ngy * width + ngx];
      if (neighbor == -1) {  // UNKNOWN
        return true;
      }
    }
  }

  return false;
}

}  // namespace exploration
