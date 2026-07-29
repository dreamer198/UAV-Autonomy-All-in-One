/*
 * This file is part of the UAV Autonomy All-in-One Fast-Planner integration.
 *
 * Fast-Planner is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as distributed with this
 * repository.
 */

#ifndef _COLLISION_RANGE_H_
#define _COLLISION_RANGE_H_

#include <cstddef>

namespace fast_planner {

enum class CollisionRangeState {
  CLEAR,
  BOUNDED,
  ENDS_IN_OBSTACLE,
  INVALID
};

inline CollisionRangeState classifyCollisionRange(
    std::size_t start_count, std::size_t end_count) {
  if (start_count == 0U) {
    return end_count == 0U ? CollisionRangeState::CLEAR
                           : CollisionRangeState::INVALID;
  }
  if (start_count == end_count) return CollisionRangeState::BOUNDED;
  if (start_count == end_count + 1U) {
    return CollisionRangeState::ENDS_IN_OBSTACLE;
  }
  return CollisionRangeState::INVALID;
}

}  // namespace fast_planner

#endif
