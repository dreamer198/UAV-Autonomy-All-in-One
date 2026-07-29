#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace sim2real_fast_adapter {

// A visualization-only copy of Fast's PointCloud2 voxelization. Only external
// sensor points are inserted here; adapter-private geometry such as the
// virtual floor remains in the native SDF map.
class RealObstacleVisualizationMap {
 public:
  RealObstacleVisualizationMap(
      const Eigen::Vector3d& origin, const Eigen::Vector3d& size,
      const Eigen::Vector3d& local_update_range, double resolution,
      double horizontal_inflation, double vertical_inflation,
      bool accumulate)
      : origin_(origin),
        size_(size),
        local_update_range_(local_update_range),
        resolution_(resolution),
        accumulate_(accumulate) {
    if (!origin_.allFinite() || !size_.allFinite() ||
        !local_update_range_.allFinite() || !std::isfinite(resolution_) ||
        !std::isfinite(horizontal_inflation) ||
        !std::isfinite(vertical_inflation) || resolution_ <= 0.0 ||
        horizontal_inflation < 0.0 || vertical_inflation < 0.0 ||
        (size_.array() <= 0.0).any() ||
        (local_update_range_.array() <= 0.0).any()) {
      throw std::invalid_argument(
          "invalid real-obstacle visualization map geometry");
    }

    uint64_t voxel_count = 1U;
    for (int axis = 0; axis < 3; ++axis) {
      const double axis_voxels = std::ceil(size_(axis) / resolution_);
      if (!std::isfinite(axis_voxels) || axis_voxels < 1.0 ||
          axis_voxels >
              static_cast<double>(std::numeric_limits<int>::max())) {
        throw std::overflow_error(
            "visualization map axis voxel count is invalid");
      }
      voxel_counts_(axis) = static_cast<int>(axis_voxels);
      const uint64_t axis_count =
          static_cast<uint64_t>(voxel_counts_(axis));
      if (voxel_count >
          std::numeric_limits<uint64_t>::max() / axis_count) {
        throw std::overflow_error("visualization map voxel count overflow");
      }
      voxel_count *= axis_count;
    }
    if (voxel_count >
        static_cast<uint64_t>(std::numeric_limits<size_t>::max())) {
      throw std::overflow_error(
          "visualization map exceeds the process address space");
    }

    const double horizontal_steps =
        std::ceil(horizontal_inflation / resolution_);
    const double vertical_steps =
        std::ceil(vertical_inflation / resolution_);
    if (horizontal_steps >
            static_cast<double>(std::numeric_limits<int>::max()) ||
        vertical_steps >
            static_cast<double>(std::numeric_limits<int>::max())) {
      throw std::overflow_error("visualization inflation step overflow");
    }
    horizontal_inflation_steps_ = static_cast<int>(horizontal_steps);
    vertical_inflation_steps_ = static_cast<int>(vertical_steps);

    occupancy_.assign(static_cast<size_t>(voxel_count), uint8_t{0});
    inflated_occupancy_.assign(
        static_cast<size_t>(voxel_count), uint8_t{0});
  }

  bool update(const std::vector<Eigen::Vector3d>& sensor_points,
              const Eigen::Vector3d& sensor_position,
              std::string* reason = nullptr) {
    if (!sensor_position.allFinite()) {
      if (reason) *reason = "visualization sensor position is not finite";
      return false;
    }

    if (!accumulate_) {
      std::fill(occupancy_.begin(), occupancy_.end(), uint8_t{0});
      std::fill(
          inflated_occupancy_.begin(), inflated_occupancy_.end(),
          uint8_t{0});
      occupied_addresses_.clear();
      inflated_addresses_.clear();
    }

    for (const Eigen::Vector3d& point : sensor_points) {
      if (!point.allFinite()) continue;
      const Eigen::Vector3d offset = point - sensor_position;
      if ((offset.array().abs() >= local_update_range_.array()).any()) {
        continue;
      }

      const Eigen::Vector3i point_index = positionToIndex(point);
      if (insideMap(point_index)) {
        mark(point_index, &occupancy_, &occupied_addresses_);
      }
      for (int dx = -horizontal_inflation_steps_;
           dx <= horizontal_inflation_steps_; ++dx) {
        for (int dy = -horizontal_inflation_steps_;
             dy <= horizontal_inflation_steps_; ++dy) {
          for (int dz = -vertical_inflation_steps_;
               dz <= vertical_inflation_steps_; ++dz) {
            const Eigen::Vector3i inflated_index =
                point_index + Eigen::Vector3i(dx, dy, dz);
            if (!insideMap(inflated_index)) continue;
            mark(
                inflated_index, &inflated_occupancy_,
                &inflated_addresses_);
          }
        }
      }
    }
    return true;
  }

  std::vector<Eigen::Vector3d> occupancyPoints(
      const Eigen::Vector3d& sensor_position) const {
    return localPoints(occupied_addresses_, sensor_position);
  }

  std::vector<Eigen::Vector3d> inflatedOccupancyPoints(
      const Eigen::Vector3d& sensor_position) const {
    return localPoints(inflated_addresses_, sensor_position);
  }

  bool inflatedObstacleWithin(
      const Eigen::Vector3d& position, double clearance,
      double* nearest_distance = nullptr) const {
    if (nearest_distance != nullptr) {
      *nearest_distance = std::numeric_limits<double>::infinity();
    }
    if (!position.allFinite() || !std::isfinite(clearance) ||
        clearance < 0.0) {
      if (nearest_distance != nullptr) *nearest_distance = 0.0;
      return true;
    }

    const Eigen::Vector3i centre = positionToIndex(position);
    const int radius =
        std::max(1, static_cast<int>(std::ceil(clearance / resolution_)) + 1);
    double nearest = std::numeric_limits<double>::infinity();
    for (int dx = -radius; dx <= radius; ++dx) {
      for (int dy = -radius; dy <= radius; ++dy) {
        for (int dz = -radius; dz <= radius; ++dz) {
          const Eigen::Vector3i index =
              centre + Eigen::Vector3i(dx, dy, dz);
          if (!insideMap(index) ||
              inflated_occupancy_[address(index)] == 0U) {
            continue;
          }
          nearest = std::min(
              nearest, (indexToPosition(index) - position).norm());
        }
      }
    }
    if (nearest_distance != nullptr) *nearest_distance = nearest;
    return nearest <= clearance;
  }

 private:
  Eigen::Vector3i positionToIndex(const Eigen::Vector3d& position) const {
    Eigen::Vector3i index;
    for (int axis = 0; axis < 3; ++axis) {
      index(axis) = static_cast<int>(
          std::floor((position(axis) - origin_(axis)) / resolution_));
    }
    return index;
  }

  Eigen::Vector3d indexToPosition(const Eigen::Vector3i& index) const {
    return origin_ +
        (index.cast<double>() + Eigen::Vector3d::Constant(0.5)) *
            resolution_;
  }

  bool insideMap(const Eigen::Vector3i& index) const {
    return (index.array() >= 0).all() &&
        (index.array() < voxel_counts_.array()).all();
  }

  size_t address(const Eigen::Vector3i& index) const {
    return (
        static_cast<size_t>(index.x()) *
            static_cast<size_t>(voxel_counts_.y()) +
        static_cast<size_t>(index.y())) *
            static_cast<size_t>(voxel_counts_.z()) +
        static_cast<size_t>(index.z());
  }

  Eigen::Vector3i indexFromAddress(size_t value) const {
    const size_t yz =
        static_cast<size_t>(voxel_counts_.y()) *
        static_cast<size_t>(voxel_counts_.z());
    const int x = static_cast<int>(value / yz);
    value %= yz;
    const int y =
        static_cast<int>(value / static_cast<size_t>(voxel_counts_.z()));
    const int z =
        static_cast<int>(value % static_cast<size_t>(voxel_counts_.z()));
    return Eigen::Vector3i(x, y, z);
  }

  void mark(const Eigen::Vector3i& index, std::vector<uint8_t>* buffer,
            std::vector<size_t>* addresses) {
    const size_t value = address(index);
    if ((*buffer)[value] != 0U) return;
    (*buffer)[value] = 1U;
    addresses->push_back(value);
  }

  void localIndexBounds(const Eigen::Vector3d& sensor_position,
                        Eigen::Vector3i* minimum,
                        Eigen::Vector3i* maximum) const {
    *minimum =
        positionToIndex(sensor_position - local_update_range_);
    *maximum =
        positionToIndex(sensor_position + local_update_range_);
    *minimum = minimum->cwiseMax(Eigen::Vector3i::Zero());
    *maximum =
        maximum->cwiseMin(voxel_counts_ - Eigen::Vector3i::Ones());
  }

  std::vector<Eigen::Vector3d> localPoints(
      const std::vector<size_t>& addresses,
      const Eigen::Vector3d& sensor_position) const {
    if (!sensor_position.allFinite()) return {};
    Eigen::Vector3i minimum;
    Eigen::Vector3i maximum;
    localIndexBounds(sensor_position, &minimum, &maximum);
    std::vector<Eigen::Vector3d> points;
    points.reserve(addresses.size());
    for (const size_t value : addresses) {
      const Eigen::Vector3i index = indexFromAddress(value);
      if ((index.array() < minimum.array()).any() ||
          (index.array() > maximum.array()).any()) {
        continue;
      }
      points.push_back(indexToPosition(index));
    }
    return points;
  }

  Eigen::Vector3d origin_;
  Eigen::Vector3d size_;
  Eigen::Vector3d local_update_range_;
  Eigen::Vector3i voxel_counts_ = Eigen::Vector3i::Zero();
  double resolution_ = 0.0;
  int horizontal_inflation_steps_ = 0;
  int vertical_inflation_steps_ = 0;
  bool accumulate_ = false;
  std::vector<uint8_t> occupancy_;
  std::vector<uint8_t> inflated_occupancy_;
  std::vector<size_t> occupied_addresses_;
  std::vector<size_t> inflated_addresses_;
};

}  // namespace sim2real_fast_adapter
