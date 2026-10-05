/**
 * @file frontier_clustering.cpp
 * @brief Frontier clustering implementation (M6).
 */

#include "exploration/frontier_detector.hpp"
#include <queue>
#include <set>

namespace exploration {

void FrontierCluster::computeCentroid(float resolution, float origin_x, float origin_y)
{
  if (cells.empty()) {
    centroid_x = 0.0f;
    centroid_y = 0.0f;
    return;
  }

  float sum_x = 0.0f, sum_y = 0.0f;
  for (const auto& cell : cells) {
    float x = origin_x + (cell.gx + 0.5f) * resolution;
    float y = origin_y + (cell.gy + 0.5f) * resolution;
    sum_x += x;
    sum_y += y;
  }

  centroid_x = sum_x / cells.size();
  centroid_y = sum_y / cells.size();
}

std::vector<FrontierCluster> FrontierClustering::clusterFrontiers(
  const std::vector<FrontierCell>& frontier_cells,
  uint32_t width, uint32_t height)
{
  if (frontier_cells.empty()) {
    return {};
  }

  // Create a map of frontier cells for fast lookup
  std::set<std::pair<uint32_t, uint32_t>> frontier_set;
  std::vector<bool> is_frontier_map(width * height, false);

  for (const auto& cell : frontier_cells) {
    frontier_set.insert({cell.gx, cell.gy});
    is_frontier_map[cell.gy * width + cell.gx] = true;
  }

  // Visited tracking
  std::vector<std::vector<int>> visited(height, std::vector<int>(width, 0));

  std::vector<FrontierCluster> clusters;
  uint32_t cluster_id = 0;

  // BFS for each unvisited frontier cell
  for (const auto& cell : frontier_cells) {
    if (visited[cell.gy][cell.gx] == 0) {
      FrontierCluster cluster(cluster_id++);
      bfs(cell.gx, cell.gy, visited, frontier_cells, width, height, cluster, is_frontier_map);
      clusters.push_back(cluster);
    }
  }

  return clusters;
}

void FrontierClustering::bfs(
  uint32_t start_gx, uint32_t start_gy,
  std::vector<std::vector<int>>& visited,
  const std::vector<FrontierCell>& frontier_cells,
  uint32_t width, uint32_t height,
  FrontierCluster& cluster,
  const std::vector<bool>& is_frontier_map)
{
  std::queue<std::pair<uint32_t, uint32_t>> q;
  q.push({start_gx, start_gy});
  visited[start_gy][start_gx] = 1;

  while (!q.empty()) {
    auto [gx, gy] = q.front();
    q.pop();

    cluster.cells.emplace_back(gx, gy);

    // Check 8-connected neighbors
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) continue;

        int ngx = static_cast<int>(gx) + dx;
        int ngy = static_cast<int>(gy) + dy;

        if (ngx >= 0 && ngx < static_cast<int>(width) &&
            ngy >= 0 && ngy < static_cast<int>(height)) {
          if (visited[ngy][ngx] == 0 && is_frontier_map[ngy * width + ngx]) {
            visited[ngy][ngx] = 1;
            q.push({static_cast<uint32_t>(ngx), static_cast<uint32_t>(ngy)});
          }
        }
      }
    }
  }
}

}  // namespace exploration
