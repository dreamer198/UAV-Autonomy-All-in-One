#ifndef PLAN_MANAGE_PLANNING_RECOVERY_UTILS_H_
#define PLAN_MANAGE_PLANNING_RECOVERY_UTILS_H_

#include <cmath>
#include <cstddef>

namespace diff_planner
{
  namespace planning_recovery_utils
  {
    inline bool isLocalTrajectoryUsable(const double now,
                                        const double start_time,
                                        const double duration)
    {
      if (!std::isfinite(now) || !std::isfinite(start_time) ||
          !std::isfinite(duration) || duration <= 0.0)
      {
        return false;
      }

      const double elapsed = now - start_time;
      return elapsed >= 0.0 && elapsed < duration;
    }

    template <typename IsOccupied>
    bool findFreeTimeByBacktracking(const double lower_time,
                                    const double candidate_time,
                                    const double sample_step,
                                    IsOccupied is_occupied,
                                    double &free_time)
    {
      if (!std::isfinite(lower_time) || !std::isfinite(candidate_time) ||
          !std::isfinite(sample_step) || sample_step <= 0.0 ||
          candidate_time < lower_time)
      {
        return false;
      }

      const double time_range = candidate_time - lower_time;
      const std::size_t sample_count =
          static_cast<std::size_t>(std::ceil(time_range / sample_step));

      for (std::size_t i = 0; i <= sample_count; ++i)
      {
        const double offset = std::fmin(time_range, i * sample_step);
        const double sample_time = candidate_time - offset;
        if (!is_occupied(sample_time))
        {
          free_time = sample_time;
          return true;
        }
      }

      return false;
    }
  } // namespace planning_recovery_utils
} // namespace diff_planner

#endif // PLAN_MANAGE_PLANNING_RECOVERY_UTILS_H_
