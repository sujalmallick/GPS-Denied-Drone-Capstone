/**
 * @file frontier_selector.cpp
 * @brief Frontier selection implementation (M7).
 */

#include "exploration/frontier_detector.hpp"
#include <cmath>
#include <algorithm>

namespace exploration {

FrontierSelector::SelectionResult FrontierSelector::selectBestFrontier(
  const std::vector<FrontierCluster>& clusters,
  float drone_x, float drone_y,
  float distance_cost_weight,
  uint32_t min_cluster_size)
{
  // Filter valid clusters
  std::vector<const FrontierCluster*> valid_clusters;
  for (const auto& cluster : clusters) {
    if (cluster.cells.size() >= min_cluster_size) {
      valid_clusters.push_back(&cluster);
    }
  }

  if (valid_clusters.empty()) {
    return SelectionResult(0, 0.0f, false);  // No valid clusters
  }

  // Compute scores for all valid clusters
  uint32_t best_cluster_id = 0;
  float best_score = -1e6f;

  for (const auto* cluster : valid_clusters) {
    // Information gain (cluster size)
    float info_gain = static_cast<float>(cluster->getInformationGain());

    // Distance cost
    float distance = computeDistance(drone_x, drone_y, cluster->centroid_x, cluster->centroid_y);
    float distance_cost = distance * distance_cost_weight;

    // Scoring formula: score = info_gain - distance_cost
    float score = info_gain - distance_cost;

    if (score > best_score) {
      best_score = score;
      best_cluster_id = cluster->cluster_id;
    }
  }

  return SelectionResult(best_cluster_id, best_score, true);
}

float FrontierSelector::computeDistance(
  float drone_x, float drone_y,
  float cluster_cx, float cluster_cy)
{
  float dx = cluster_cx - drone_x;
  float dy = cluster_cy - drone_y;
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace exploration
