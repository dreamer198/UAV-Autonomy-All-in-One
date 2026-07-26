#ifndef PATH_SEARCHING_ENDPOINT_ADJUSTMENT_UTILS_H_
#define PATH_SEARCHING_ENDPOINT_ADJUSTMENT_UTILS_H_

#include <algorithm>
#include <cmath>

#include <Eigen/Eigen>

namespace path_searching
{
  namespace endpoint_adjustment
  {
    template <typename OccupancyAt>
    bool moveAwayFromOccupiedSegment(
        const Eigen::Vector3d &occupied_point,
        const Eigen::Vector3d &opposite_segment_endpoint,
        const double step_size,
        const double max_search_distance,
        OccupancyAt occupancy_at,
        Eigen::Vector3d &adjusted_point)
    {
      if (!occupied_point.allFinite() ||
          !opposite_segment_endpoint.allFinite() ||
          !std::isfinite(step_size) || step_size <= 0.0 ||
          !std::isfinite(max_search_distance) ||
          max_search_distance <= 0.0)
      {
        return false;
      }

      // A-star is called with the two nominally free samples immediately
      // outside a collision interval. Voxel quantization can still map either
      // sample into an occupied cell. Move that endpoint farther away from
      // the collision interval; moving it toward the opposite endpoint would
      // search through the obstacle itself.
      const Eigen::Vector3d delta =
          occupied_point - opposite_segment_endpoint;
      const double distance = delta.norm();
      if (!std::isfinite(distance) || distance <= 1e-9)
      {
        return false;
      }

      const Eigen::Vector3d direction = delta / distance;
      const int step_count =
          std::max(1, static_cast<int>(
                          std::ceil(max_search_distance / step_size)));
      for (int step = 1; step <= step_count; ++step)
      {
        const double travel = std::min(
            max_search_distance, static_cast<double>(step) * step_size);
        const Eigen::Vector3d candidate =
            occupied_point + direction * travel;
        const int occupancy = occupancy_at(candidate);
        if (occupancy < 0)
        {
          // Never turn an endpoint-adjustment fallback into a search outside
          // the valid rolling map or virtual fence.
          return false;
        }
        if (occupancy == 0)
        {
          adjusted_point = candidate;
          return true;
        }
      }

      return false;
    }
  } // namespace endpoint_adjustment
} // namespace path_searching

#endif // PATH_SEARCHING_ENDPOINT_ADJUSTMENT_UTILS_H_
