/*
 * This file is part of the UAV Autonomy All-in-One Fast-Planner integration.
 *
 * Fast-Planner is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as distributed with this
 * repository.
 */

#ifndef _TOPOLOGY_SAFETY_H_
#define _TOPOLOGY_SAFETY_H_

#include <algorithm>
#include <cmath>
#include <vector>

namespace fast_planner {

inline std::vector<double> sampleTimesIncludingEnd(
    double start, double end, double maximum_step) {
  std::vector<double> samples;
  if (!std::isfinite(start) || !std::isfinite(end) ||
      !std::isfinite(maximum_step) || end < start || maximum_step <= 0.0) {
    return samples;
  }

  const double duration = end - start;
  if (duration <= 1.0e-9) {
    samples.push_back(start);
    return samples;
  }

  const int segment_count =
      std::max(1, static_cast<int>(std::ceil(duration / maximum_step)));
  samples.reserve(static_cast<std::size_t>(segment_count) + 1U);
  for (int index = 0; index <= segment_count; ++index) {
    samples.push_back(
        start + duration * static_cast<double>(index) /
                    static_cast<double>(segment_count));
  }
  return samples;
}

inline bool violatesRequiredClearance(double distance, double clearance) {
  return !std::isfinite(distance) || !std::isfinite(clearance) ||
      clearance <= 0.0 || distance <= clearance;
}

inline bool isObservedClearance(double distance) {
  // SDFMap initializes and clears cells outside its latest ESDF update box
  // to 10000. That sentinel is not an obstacle-free measurement.
  return std::isfinite(distance) && distance > -10000.0 &&
      distance < 9999.0;
}

inline double topologyEdgeClearance(
    double required_clearance, double map_resolution) {
  if (!std::isfinite(required_clearance) ||
      !std::isfinite(map_resolution) ||
      required_clearance <= 0.0 || map_resolution <= 0.0) {
    return 0.0;
  }

  /*
   * Collision-range guards can lie exactly on the clearance isosurface while
   * lineVisib() treats equality as occupied. Leave only a floating-point
   * epsilon for that equality. Subtracting a whole voxel makes a 0.3 m PRM
   * silently accept 0.2 m edges in the forest map, which the final trajectory
   * safety gate must then reject.
   */
  const double equality_epsilon =
      std::max(1.0e-6, 1.0e-3 * map_resolution);
  return std::max(
      map_resolution, required_clearance - equality_epsilon);
}

/*
 * A rolling map update can put the measured trajectory start a few
 * centimetres inside the configured clearance even though the vehicle is not
 * in an occupied voxel. Rejecting that first sample makes every candidate
 * invalid, including one that immediately moves away from the obstacle.
 *
 * This validator keeps the configured clearance for the whole trajectory,
 * except for one short prefix that starts below it. The prefix must remain
 * collision-free, may not lose more than the supplied tolerance from the best
 * clearance seen so far, and must recover the full clearance within the
 * supplied travelled distance.
 */
class InitialClearanceEscape {
public:
  InitialClearanceEscape(
      double initial_clearance, double required_clearance,
      double regression_tolerance, double maximum_escape_distance)
      : valid_(std::isfinite(initial_clearance) &&
               std::isfinite(required_clearance) &&
               std::isfinite(regression_tolerance) &&
               std::isfinite(maximum_escape_distance) &&
               initial_clearance > 0.0 && required_clearance > 0.0 &&
               regression_tolerance >= 0.0 &&
               maximum_escape_distance > 0.0),
        full_clearance_reached_(
            valid_ && initial_clearance >= required_clearance),
        required_clearance_(required_clearance),
        regression_tolerance_(regression_tolerance),
        maximum_escape_distance_(maximum_escape_distance),
        best_clearance_(initial_clearance) {}

  bool accept(double clearance, double travelled_distance) {
    if (!valid_ || !std::isfinite(clearance) ||
        !std::isfinite(travelled_distance) || clearance <= 0.0 ||
        travelled_distance < 0.0) {
      valid_ = false;
      return false;
    }

    if (full_clearance_reached_) {
      if (clearance < required_clearance_) {
        valid_ = false;
        return false;
      }
      best_clearance_ = std::max(best_clearance_, clearance);
      return true;
    }

    if (travelled_distance > maximum_escape_distance_ ||
        clearance + regression_tolerance_ < best_clearance_) {
      valid_ = false;
      return false;
    }

    best_clearance_ = std::max(best_clearance_, clearance);
    if (clearance >= required_clearance_) {
      full_clearance_reached_ = true;
    }
    return true;
  }

  bool complete() const { return valid_ && full_clearance_reached_; }
  bool escaping() const { return valid_ && !full_clearance_reached_; }

private:
  bool valid_;
  bool full_clearance_reached_;
  double required_clearance_;
  double regression_tolerance_;
  double maximum_escape_distance_;
  double best_clearance_;
};

}  // namespace fast_planner

#endif
