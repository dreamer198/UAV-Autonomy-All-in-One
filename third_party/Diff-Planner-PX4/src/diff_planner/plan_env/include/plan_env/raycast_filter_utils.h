#ifndef PLAN_ENV_RAYCAST_FILTER_UTILS_H_
#define PLAN_ENV_RAYCAST_FILTER_UTILS_H_

#include <cmath>

#include <Eigen/Core>

namespace plan_env
{
namespace raycast_filter
{
inline bool isValidMinimumRayLength(const double minimum_ray_length)
{
  return std::isfinite(minimum_ray_length) && minimum_ray_length >= 0.0;
}

inline bool shouldProcessRay(const Eigen::Vector3d &camera_position,
                             const Eigen::Vector3d &endpoint,
                             const double minimum_ray_length)
{
  if (!camera_position.allFinite() || !endpoint.allFinite() ||
      !isValidMinimumRayLength(minimum_ray_length))
  {
    return false;
  }

  const Eigen::Vector3d ray = endpoint - camera_position;
  return ray.squaredNorm() >=
         minimum_ray_length * minimum_ray_length;
}
} // namespace raycast_filter
} // namespace plan_env

#endif // PLAN_ENV_RAYCAST_FILTER_UTILS_H_
