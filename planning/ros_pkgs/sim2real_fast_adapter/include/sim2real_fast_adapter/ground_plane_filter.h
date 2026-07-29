#pragma once

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace sim2real_fast_adapter {

inline bool fitDominantGroundPlane(
    const std::vector<Eigen::Vector3d>& points,
    double candidate_minimum_z, double candidate_maximum_z,
    double distance_tolerance, double maximum_tilt_radians,
    size_t minimum_inliers, double minimum_xy_span,
    int ransac_iterations, Eigen::Vector4d* plane) {
  if (plane == nullptr || !std::isfinite(candidate_minimum_z) ||
      !std::isfinite(candidate_maximum_z) ||
      !std::isfinite(distance_tolerance) ||
      !std::isfinite(maximum_tilt_radians) ||
      !std::isfinite(minimum_xy_span) ||
      candidate_maximum_z <= candidate_minimum_z ||
      distance_tolerance <= 0.0 || maximum_tilt_radians <= 0.0 ||
      maximum_tilt_radians >= 0.5 * std::acos(-1.0) ||
      minimum_inliers < 3U || minimum_xy_span <= 0.0 ||
      ransac_iterations <= 0) {
    return false;
  }

  std::vector<size_t> candidates;
  candidates.reserve(points.size());
  for (size_t index = 0; index < points.size(); ++index) {
    const Eigen::Vector3d& point = points[index];
    if (point.allFinite() && point.z() >= candidate_minimum_z &&
        point.z() <= candidate_maximum_z) {
      candidates.push_back(index);
    }
  }
  if (candidates.size() < minimum_inliers) return false;

  const double minimum_normal_z = std::cos(maximum_tilt_radians);
  size_t best_count = 0U;
  double best_error = std::numeric_limits<double>::infinity();
  Eigen::Vector3d best_normal = Eigen::Vector3d::UnitZ();
  double best_offset = 0.0;
  uint64_t state = 0x9e3779b97f4a7c15ULL;
  const auto next_candidate = [&candidates, &state]() {
    state = state * 6364136223846793005ULL + 1442695040888963407ULL;
    return candidates[static_cast<size_t>(state % candidates.size())];
  };

  for (int iteration = 0; iteration < ransac_iterations; ++iteration) {
    const size_t first = next_candidate();
    size_t second = next_candidate();
    size_t third = next_candidate();
    for (int retry = 0; retry < 4 && second == first; ++retry) {
      second = next_candidate();
    }
    for (int retry = 0;
         retry < 4 && (third == first || third == second); ++retry) {
      third = next_candidate();
    }
    if (first == second || first == third || second == third) continue;

    Eigen::Vector3d normal =
        (points[second] - points[first])
            .cross(points[third] - points[first]);
    const double norm = normal.norm();
    if (!std::isfinite(norm) || norm < 1e-9) continue;
    normal /= norm;
    if (normal.z() < 0.0) normal = -normal;
    if (normal.z() < minimum_normal_z) continue;
    const double offset = -normal.dot(points[first]);

    size_t count = 0U;
    double error = 0.0;
    for (const size_t index : candidates) {
      const double distance =
          std::abs(normal.dot(points[index]) + offset);
      if (distance > distance_tolerance) continue;
      ++count;
      error += distance;
    }
    if (count > best_count || (count == best_count && error < best_error)) {
      best_count = count;
      best_error = error;
      best_normal = normal;
      best_offset = offset;
    }
  }
  if (best_count < minimum_inliers) return false;

  for (int refinement = 0; refinement < 3; ++refinement) {
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    size_t count = 0U;
    for (const size_t index : candidates) {
      if (std::abs(best_normal.dot(points[index]) + best_offset) <=
          distance_tolerance) {
        centroid += points[index];
        ++count;
      }
    }
    if (count < minimum_inliers) return false;
    centroid /= static_cast<double>(count);

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const size_t index : candidates) {
      if (std::abs(best_normal.dot(points[index]) + best_offset) >
          distance_tolerance) {
        continue;
      }
      const Eigen::Vector3d centered = points[index] - centroid;
      covariance.noalias() += centered * centered.transpose();
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success) return false;
    Eigen::Vector3d refined_normal = solver.eigenvectors().col(0);
    if (refined_normal.z() < 0.0) refined_normal = -refined_normal;
    if (!refined_normal.allFinite() ||
        refined_normal.z() < minimum_normal_z) {
      return false;
    }
    best_normal = refined_normal;
    best_offset = -best_normal.dot(centroid);
  }

  size_t final_count = 0U;
  double minimum_x = std::numeric_limits<double>::infinity();
  double maximum_x = -std::numeric_limits<double>::infinity();
  double minimum_y = std::numeric_limits<double>::infinity();
  double maximum_y = -std::numeric_limits<double>::infinity();
  for (const size_t index : candidates) {
    if (std::abs(best_normal.dot(points[index]) + best_offset) >
        distance_tolerance) {
      continue;
    }
    ++final_count;
    minimum_x = std::min(minimum_x, points[index].x());
    maximum_x = std::max(maximum_x, points[index].x());
    minimum_y = std::min(minimum_y, points[index].y());
    maximum_y = std::max(maximum_y, points[index].y());
  }
  if (final_count < minimum_inliers ||
      maximum_x - minimum_x < minimum_xy_span ||
      maximum_y - minimum_y < minimum_xy_span) {
    return false;
  }

  plane->head<3>() = best_normal;
  (*plane)(3) = best_offset;
  return plane->allFinite();
}

inline std::vector<Eigen::Vector3d> removeGroundPlanePoints(
    const std::vector<Eigen::Vector3d>& points,
    const Eigen::Vector4d& plane, double distance_tolerance,
    size_t* removed_points = nullptr) {
  if (removed_points != nullptr) *removed_points = 0U;
  if (!plane.allFinite() || !std::isfinite(distance_tolerance) ||
      distance_tolerance <= 0.0 ||
      plane.head<3>().norm() < 1e-9) {
    return points;
  }

  const Eigen::Vector3d normal = plane.head<3>().normalized();
  const double offset = plane(3) / plane.head<3>().norm();
  std::vector<Eigen::Vector3d> output;
  output.reserve(points.size());
  for (const Eigen::Vector3d& point : points) {
    if (point.allFinite() &&
        std::abs(normal.dot(point) + offset) <= distance_tolerance) {
      if (removed_points != nullptr) ++(*removed_points);
      continue;
    }
    output.push_back(point);
  }
  return output;
}

}  // namespace sim2real_fast_adapter
