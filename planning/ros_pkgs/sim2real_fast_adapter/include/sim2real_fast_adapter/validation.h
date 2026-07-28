#pragma once

#include <Eigen/Geometry>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/time.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace sim2real_fast_adapter {

inline bool finite(double value) { return std::isfinite(value); }

inline bool finitePoint(const geometry_msgs::Point& point) {
  return finite(point.x) && finite(point.y) && finite(point.z);
}

inline bool measurementStampIsCurrent(const ros::Time& stamp,
                                      const ros::Time& now,
                                      double maximum_age,
                                      const ros::Time& previous_stamp,
                                      double future_tolerance = 0.1) {
  if (stamp.isZero() || now.isZero() || !finite(maximum_age) ||
      maximum_age <= 0.0 || !finite(future_tolerance) ||
      future_tolerance < 0.0 ||
      (!previous_stamp.isZero() && stamp <= previous_stamp)) {
    return false;
  }
  const double age = (now - stamp).toSec();
  return finite(age) && age <= maximum_age && age >= -future_tolerance;
}

inline bool finitePose(const geometry_msgs::PoseStamped& pose, bool constrain_yaw,
                       std::string* reason) {
  if (pose.header.frame_id != "world") {
    if (reason) *reason = "goal frame_id must be world";
    return false;
  }
  if (!finitePoint(pose.pose.position)) {
    if (reason) *reason = "goal position must be finite";
    return false;
  }
  const auto& q = pose.pose.orientation;
  if (!finite(q.x) || !finite(q.y) || !finite(q.z) || !finite(q.w)) {
    if (reason) *reason = "goal quaternion must be finite";
    return false;
  }
  const double norm2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  if (!constrain_yaw) {
    if (norm2 > 1e-12) {
      if (reason) *reason = "unconstrained yaw must use the zero quaternion";
      return false;
    }
    return true;
  }
  if (!finite(norm2) || norm2 < 0.9801 || norm2 > 1.0201) {
    if (reason) *reason = "constrained yaw requires a finite unit quaternion";
    return false;
  }
  return true;
}

inline bool goalInsideMapBounds(const geometry_msgs::Point& point,
                                const Eigen::Vector3d& origin,
                                const Eigen::Vector3d& size, double inflation,
                                std::string* reason) {
  if (!finitePoint(point) || !origin.allFinite() || !size.allFinite() ||
      !finite(inflation) || (size.array() <= 0.0).any() || inflation < 0.0) {
    if (reason) *reason = "invalid goal or map geometry";
    return false;
  }
  const Eigen::Vector3d p(point.x, point.y, point.z);
  // local_update_range is a sensor-processing window around the current
  // vehicle, not a global-map boundary requirement. SDFMap already clips
  // that window at the fixed AABB. Only keep the physical obstacle-inflation
  // clearance so the vehicle center cannot end immediately against the
  // out-of-map region, which Fast treats as occupied.
  const Eigen::Vector3d margin = Eigen::Vector3d::Constant(inflation);
  const Eigen::Vector3d safe_min = origin + margin;
  const Eigen::Vector3d safe_max = origin + size - margin;
  if ((safe_min.array() > safe_max.array()).any()) {
    if (reason) *reason = "map is smaller than its obstacle-inflation margin";
    return false;
  }
  if ((p.array() < safe_min.array()).any() || (p.array() > safe_max.array()).any()) {
    if (reason) {
      *reason = "goal is outside the fixed map bounds after obstacle-inflation margin";
    }
    return false;
  }
  return true;
}

inline Eigen::Vector3d bodyVelocityToWorld(const Eigen::Quaterniond& body_in_world,
                                           const Eigen::Vector3d& body_velocity) {
  return body_in_world.normalized() * body_velocity;
}

inline bool finiteTrajectorySetpoint(
    const Eigen::Vector3d& position, const Eigen::Vector3d& velocity,
    const Eigen::Vector3d& acceleration, double yaw, double yaw_rate) {
  // Fast's manager/max_vel and manager/max_acc are optimization parameters,
  // not a controller wire-contract limit. Topological refinement may
  // transiently exceed those nominal values. The plugin boundary therefore
  // rejects malformed/non-finite output, while the selected controller and
  // flight configuration remain responsible for their own operating limits.
  return position.allFinite() && velocity.allFinite() &&
      acceleration.allFinite() && finite(yaw) && finite(yaw_rate);
}

inline bool pointInsideBodyExclusionCylinder(
    const Eigen::Vector3d& point_world,
    const Eigen::Vector3d& body_position_world,
    const Eigen::Quaterniond& body_in_world, double radius,
    double minimum_z, double maximum_z) {
  if (!point_world.allFinite() || !body_position_world.allFinite() ||
      !body_in_world.coeffs().allFinite() || body_in_world.norm() < 1e-6 ||
      !finite(radius) || !finite(minimum_z) || !finite(maximum_z) ||
      radius <= 0.0 || minimum_z >= maximum_z) {
    return false;
  }
  const Eigen::Vector3d point_body =
      body_in_world.normalized().conjugate() *
      (point_world - body_position_world);
  return point_body.head<2>().squaredNorm() <= radius * radius &&
      point_body.z() >= minimum_z && point_body.z() <= maximum_z;
}

inline double quaternionYaw(const Eigen::Quaterniond& orientation) {
  const Eigen::Quaterniond q = orientation.normalized();
  return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                    1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

inline bool measuredStateSatisfiesGoal(
    const Eigen::Vector3d& measured_position,
    const Eigen::Vector3d& measured_world_velocity,
    const Eigen::Quaterniond& measured_body_in_world,
    const Eigen::Vector3d& measured_world_angular_velocity,
    const geometry_msgs::PoseStamped& goal, bool constrain_yaw,
    double position_tolerance, double velocity_tolerance,
    double yaw_tolerance, double yaw_rate_tolerance) {
  if (!measured_position.allFinite() || !measured_world_velocity.allFinite() ||
      !measured_body_in_world.coeffs().allFinite() ||
      !measured_world_angular_velocity.allFinite() ||
      !finitePoint(goal.pose.position) || !finite(position_tolerance) ||
      !finite(velocity_tolerance) || !finite(yaw_tolerance) ||
      !finite(yaw_rate_tolerance) || position_tolerance <= 0.0 ||
      velocity_tolerance <= 0.0 || yaw_tolerance <= 0.0 ||
      yaw_rate_tolerance <= 0.0 || measured_body_in_world.norm() < 1e-6) {
    return false;
  }

  const Eigen::Vector3d target(goal.pose.position.x, goal.pose.position.y,
                               goal.pose.position.z);
  if ((measured_position - target).norm() > position_tolerance ||
      measured_world_velocity.norm() > velocity_tolerance) {
    return false;
  }
  if (!constrain_yaw) return true;

  const auto& q = goal.pose.orientation;
  if (!finite(q.x) || !finite(q.y) || !finite(q.z) || !finite(q.w)) return false;
  const Eigen::Quaterniond target_orientation(q.w, q.x, q.y, q.z);
  if (target_orientation.norm() < 1e-6) return false;

  const double yaw_error =
      std::atan2(std::sin(quaternionYaw(target_orientation) -
                         quaternionYaw(measured_body_in_world)),
                 std::cos(quaternionYaw(target_orientation) -
                          quaternionYaw(measured_body_in_world)));
  return std::abs(yaw_error) <= yaw_tolerance &&
      std::abs(measured_world_angular_velocity.z()) <= yaw_rate_tolerance;
}

inline bool buildVirtualFloor(
    const Eigen::Vector3d& vehicle_position,
    const Eigen::Vector3d& map_origin,
    const Eigen::Vector3d& map_size,
    const Eigen::Vector3d& local_update_range,
    double resolution, double floor_height,
    std::vector<Eigen::Vector3d>* points, std::string* reason) {
  if (points == nullptr || !vehicle_position.allFinite() ||
      !map_origin.allFinite() || !map_size.allFinite() ||
      !local_update_range.allFinite() || !finite(resolution) ||
      !finite(floor_height) || resolution <= 0.0 ||
      (map_size.array() <= 0.0).any() ||
      (local_update_range.array() <= 0.0).any()) {
    if (reason) *reason = "invalid virtual-floor geometry";
    return false;
  }
  const Eigen::Vector3d map_max = map_origin + map_size;
  if (floor_height <= map_origin.z() || floor_height >= map_max.z() ||
      std::abs(floor_height - vehicle_position.z()) >= local_update_range.z()) {
    if (reason) *reason = "virtual floor is outside the active map window";
    return false;
  }

  const int64_t voxel_count_x =
      static_cast<int64_t>(std::ceil(map_size.x() / resolution));
  const int64_t voxel_count_y =
      static_cast<int64_t>(std::ceil(map_size.y() / resolution));
  if (voxel_count_x <= 0 || voxel_count_y <= 0 ||
      voxel_count_x > std::numeric_limits<int>::max() ||
      voxel_count_y > std::numeric_limits<int>::max()) {
    if (reason) *reason = "virtual-floor voxel count is invalid";
    return false;
  }

  const double min_x = vehicle_position.x() - local_update_range.x();
  const double max_x = vehicle_position.x() + local_update_range.x();
  const double min_y = vehicle_position.y() - local_update_range.y();
  const double max_y = vehicle_position.y() + local_update_range.y();
  const int64_t first_x = std::max<int64_t>(
      0, static_cast<int64_t>(std::floor((min_x - map_origin.x()) / resolution)));
  const int64_t last_x = std::min<int64_t>(
      voxel_count_x - 1,
      static_cast<int64_t>(std::floor((max_x - map_origin.x()) / resolution)));
  const int64_t first_y = std::max<int64_t>(
      0, static_cast<int64_t>(std::floor((min_y - map_origin.y()) / resolution)));
  const int64_t last_y = std::min<int64_t>(
      voxel_count_y - 1,
      static_cast<int64_t>(std::floor((max_y - map_origin.y()) / resolution)));
  if (first_x > last_x || first_y > last_y) {
    if (reason) *reason = "virtual floor does not intersect the active map window";
    return false;
  }

  const uint64_t count_x = static_cast<uint64_t>(last_x - first_x + 1);
  const uint64_t count_y = static_cast<uint64_t>(last_y - first_y + 1);
  if (count_x > std::numeric_limits<size_t>::max() / count_y) {
    if (reason) *reason = "virtual-floor point count overflow";
    return false;
  }
  points->clear();
  points->reserve(static_cast<size_t>(count_x * count_y));
  for (int64_t ix = first_x; ix <= last_x; ++ix) {
    const double x = map_origin.x() + (static_cast<double>(ix) + 0.5) * resolution;
    if (std::abs(x - vehicle_position.x()) >= local_update_range.x()) continue;
    for (int64_t iy = first_y; iy <= last_y; ++iy) {
      const double y =
          map_origin.y() + (static_cast<double>(iy) + 0.5) * resolution;
      if (std::abs(y - vehicle_position.y()) >= local_update_range.y()) continue;
      points->emplace_back(x, y, floor_height);
    }
  }
  if (points->empty()) {
    if (reason) *reason = "virtual floor contains no points";
    return false;
  }
  return true;
}

}  // namespace sim2real_fast_adapter
