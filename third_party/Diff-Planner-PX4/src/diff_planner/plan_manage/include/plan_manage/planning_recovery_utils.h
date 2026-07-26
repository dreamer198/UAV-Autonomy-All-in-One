#ifndef PLAN_MANAGE_PLANNING_RECOVERY_UTILS_H_
#define PLAN_MANAGE_PLANNING_RECOVERY_UTILS_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

#include <Eigen/Eigen>

namespace diff_planner
{
  namespace planning_recovery_utils
  {
    inline bool isNumericallySafeQuinticDuration(const double duration)
    {
      // Piece::getPos evaluates a fifth-order polynomial with powers of time.
      // Keeping t^5 below the double range prevents finite-but-extreme goals
      // from producing Inf/NaN coefficients or samples. This is a numerical
      // guard, not an operational goal-distance limit.
      constexpr double max_safe_duration = 1.0e60;
      return std::isfinite(duration) && duration > 0.0 &&
             duration <= max_safe_duration;
    }

    inline std::size_t boundedTrajectorySampleCount(
        const double duration,
        const double requested_step,
        const std::size_t max_samples)
    {
      if (!isNumericallySafeQuinticDuration(duration) ||
          !std::isfinite(requested_step) || requested_step <= 0.0 ||
          max_samples < 2)
      {
        return 0;
      }

      const double max_intervals = static_cast<double>(max_samples - 1);
      if (duration >= requested_step * max_intervals)
      {
        return max_samples;
      }
      const double intervals = std::ceil(duration / requested_step);
      return std::max<std::size_t>(
          2, static_cast<std::size_t>(intervals) + 1);
    }

    template <typename PositionAtTime>
    bool findMonotonicHorizonCrossingTime(
        const double lower_time,
        const double upper_time,
        const double horizon,
        const Eigen::Vector3d &start,
        PositionAtTime position_at_time,
        double &crossing_time,
        const std::size_t max_iterations = 96)
    {
      if (!std::isfinite(lower_time) || !std::isfinite(upper_time) ||
          upper_time < lower_time || !std::isfinite(horizon) ||
          horizon <= 0.0 || !start.allFinite() || max_iterations == 0)
      {
        return false;
      }

      const auto reaches_horizon = [&](const double time, bool &reached) {
        const Eigen::Vector3d position = position_at_time(time);
        if (!position.allFinite())
        {
          return false;
        }
        const double distance = (position - start).norm();
        if (!std::isfinite(distance))
        {
          return false;
        }
        reached = distance >= horizon;
        return true;
      };

      bool lower_reached = false;
      bool upper_reached = false;
      if (!reaches_horizon(lower_time, lower_reached) ||
          !reaches_horizon(upper_time, upper_reached) || !upper_reached)
      {
        return false;
      }
      if (lower_reached)
      {
        crossing_time = lower_time;
        return true;
      }

      double lower = lower_time;
      double upper = upper_time;
      for (std::size_t i = 0; i < max_iterations; ++i)
      {
        const double middle = lower + 0.5 * (upper - lower);
        if (!(middle > lower && middle < upper))
        {
          break;
        }
        bool middle_reached = false;
        if (!reaches_horizon(middle, middle_reached))
        {
          return false;
        }
        if (middle_reached)
        {
          upper = middle;
        }
        else
        {
          lower = middle;
        }
      }
      crossing_time = upper;
      return std::isfinite(crossing_time);
    }

    class ReplanFailureWindow
    {
    public:
      ReplanFailureWindow()
          : retry_interval_(0.1),
            timeout_(1.0),
            first_failure_time_(0.0),
            next_attempt_time_(0.0),
            last_observed_time_(0.0),
            active_(false),
            time_initialized_(false)
      {
      }

      void configure(const double retry_interval, const double timeout)
      {
        retry_interval_ = retry_interval;
        timeout_ = timeout;
      }

      void reset(const double now)
      {
        active_ = false;
        first_failure_time_ = 0.0;
        if (std::isfinite(now))
        {
          next_attempt_time_ = now;
          last_observed_time_ = now;
          time_initialized_ = true;
        }
        else
        {
          next_attempt_time_ = 0.0;
          last_observed_time_ = 0.0;
          time_initialized_ = false;
        }
      }

      bool shouldAttempt(const double now)
      {
        if (!std::isfinite(now))
        {
          return false;
        }
        handleClockRewind(now);
        last_observed_time_ = now;
        time_initialized_ = true;
        return !active_ || now + 1e-9 >= next_attempt_time_;
      }

      void recordFailure(const double now)
      {
        if (!std::isfinite(now))
        {
          return;
        }
        handleClockRewind(now);
        if (!active_)
        {
          first_failure_time_ = now;
          active_ = true;
        }
        next_attempt_time_ = now + retry_interval_;
        last_observed_time_ = now;
        time_initialized_ = true;
      }

      bool timedOut(const double now)
      {
        if (!std::isfinite(now) || !active_)
        {
          return false;
        }
        if (time_initialized_ && now + 1e-9 < last_observed_time_)
        {
          reset(now);
          return false;
        }
        last_observed_time_ = now;
        time_initialized_ = true;
        return now - first_failure_time_ + 1e-9 >= timeout_;
      }

      bool active() const
      {
        return active_;
      }

    private:
      void handleClockRewind(const double now)
      {
        if (time_initialized_ && now + 1e-9 < last_observed_time_)
        {
          reset(now);
        }
      }

      double retry_interval_;
      double timeout_;
      double first_failure_time_;
      double next_attempt_time_;
      double last_observed_time_;
      bool active_;
      bool time_initialized_;
    };

    class StuckProgressMonitor
    {
    public:
      StuckProgressMonitor()
          : progress_threshold_(0.1),
            timeout_(5.0),
            last_progress_time_(0.0),
            position_initialized_(false),
            time_initialized_(false)
      {
        last_progress_position_.setZero();
      }

      void configure(const double progress_threshold, const double timeout)
      {
        progress_threshold_ = progress_threshold;
        timeout_ = timeout;
      }

      void reset(const double now)
      {
        last_progress_time_ = std::isfinite(now) ? now : 0.0;
        position_initialized_ = false;
        time_initialized_ = std::isfinite(now);
      }

      // "Stuck" means that the vehicle itself has stopped moving.  Motion
      // toward a local/final target is deliberately not used here: a valid
      // obstacle detour can initially be lateral or even move away from the
      // final goal, while a moving local target must not hide a stationary
      // vehicle.
      bool updateVehicleMotion(const double now,
                               const Eigen::Vector3d &current_position)
      {
        if (!std::isfinite(now) || !current_position.allFinite())
        {
          return false;
        }

        if (time_initialized_ && now + 1e-9 < last_progress_time_)
        {
          reset(now);
        }

        if (!position_initialized_)
        {
          last_progress_position_ = current_position;
          last_progress_time_ = now;
          position_initialized_ = true;
          time_initialized_ = true;
          return false;
        }

        if ((current_position - last_progress_position_).norm() >=
            progress_threshold_)
        {
          last_progress_position_ = current_position;
          last_progress_time_ = now;
        }

        time_initialized_ = true;
        return now - last_progress_time_ > timeout_;
      }

    private:
      double progress_threshold_;
      double timeout_;
      double last_progress_time_;
      bool position_initialized_;
      bool time_initialized_;
      Eigen::Vector3d last_progress_position_;
    };

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

    inline bool isWithinVerticalClearance(const double z,
                                          const double virtual_ground,
                                          const double virtual_ceil,
                                          const double clearance)
    {
      if (!std::isfinite(z) || !std::isfinite(virtual_ground) ||
          !std::isfinite(virtual_ceil) || !std::isfinite(clearance) ||
          clearance < 0.0 ||
          virtual_ground + clearance >= virtual_ceil - clearance)
      {
        return false;
      }

      return z > virtual_ground + clearance &&
             z < virtual_ceil - clearance;
    }

    template <typename IsOccupied>
    bool findFreeTimeByBacktracking(const double lower_time,
                                    const double candidate_time,
                                    const double sample_step,
                                    IsOccupied is_occupied,
                                    double &free_time,
                                    const std::size_t max_evaluations = 512)
    {
      if (!std::isfinite(lower_time) || !std::isfinite(candidate_time) ||
          !std::isfinite(sample_step) || sample_step <= 0.0 ||
          candidate_time < lower_time || max_evaluations < 2)
      {
        return false;
      }

      const double time_range = candidate_time - lower_time;
      if (!std::isfinite(time_range))
      {
        return false;
      }

      const double requested_interval_count =
          std::ceil(time_range / sample_step);
      const bool use_requested_step =
          std::isfinite(requested_interval_count) &&
          requested_interval_count <
              static_cast<double>(max_evaluations);
      const std::size_t interval_count =
          use_requested_step
              ? static_cast<std::size_t>(requested_interval_count)
              : max_evaluations - 1;

      // Preserve sample_step exactly for ordinary trajectories. If the
      // requested count is too large (or overflows double), distribute the
      // fixed evaluation budget across the whole interval instead of casting
      // an out-of-range value to size_t or walking the interval indefinitely.
      for (std::size_t i = 0; i <= interval_count; ++i)
      {
        double sample_time = lower_time;
        if (i < interval_count)
        {
          if (use_requested_step)
          {
            sample_time =
                candidate_time - static_cast<double>(i) * sample_step;
          }
          else
          {
            const double alpha =
                static_cast<double>(i) /
                static_cast<double>(interval_count);
            sample_time = candidate_time - alpha * time_range;
          }
        }
        if (!is_occupied(sample_time))
        {
          free_time = sample_time;
          return true;
        }
      }

      return false;
    }

    template <typename IsOccupied, typename IsStartVoxel>
    bool isStraightLineFree(const Eigen::Vector3d &start,
                            const Eigen::Vector3d &end,
                            const double sample_step,
                            IsOccupied is_occupied,
                            IsStartVoxel is_start_voxel)
    {
      if (!start.allFinite() || !end.allFinite() ||
          !std::isfinite(sample_step) || sample_step <= 0.0)
      {
        return false;
      }

      const Eigen::Vector3d delta = end - start;
      const double distance = delta.norm();
      if (distance <= 1e-9)
      {
        return !is_occupied(end);
      }

      const std::size_t sample_count = std::max<std::size_t>(
          1, static_cast<std::size_t>(std::ceil(distance / sample_step)));
      // Every sample in the vehicle's current voxel is omitted because a map
      // update can mark that whole voxel occupied while the vehicle is still
      // inside it. Once the ray leaves that voxel, every sample (including the
      // endpoint) must be free.
      for (std::size_t i = 1; i <= sample_count; ++i)
      {
        const double alpha =
            static_cast<double>(i) / static_cast<double>(sample_count);
        const Eigen::Vector3d position = start + alpha * delta;
        if (!is_start_voxel(position) && is_occupied(position))
        {
          return false;
        }
      }
      return true;
    }

    template <typename IsOccupied>
    bool isStraightLineFree(const Eigen::Vector3d &start,
                            const Eigen::Vector3d &end,
                            const double sample_step,
                            IsOccupied is_occupied)
    {
      return isStraightLineFree(
          start, end, sample_step, is_occupied,
          [&](const Eigen::Vector3d &position) {
            return (position - start).squaredNorm() <= 1e-18;
          });
    }

    template <typename IsOccupied, typename IsStartVoxel>
    bool findNearbyFreePosition(const Eigen::Vector3d &candidate,
                                const Eigen::Vector3d &start,
                                const Eigen::Vector3d &forward_direction,
                                const double resolution,
                                const double max_search_radius,
                                const double max_target_distance,
                                IsOccupied is_occupied,
                                IsStartVoxel is_start_voxel,
                                Eigen::Vector3d &free_position)
    {
      if (!candidate.allFinite() || !start.allFinite() ||
          !forward_direction.allFinite() ||
          !std::isfinite(resolution) || resolution <= 0.0 ||
          !std::isfinite(max_search_radius) || max_search_radius <= 0.0 ||
          !std::isfinite(max_target_distance) || max_target_distance <= 0.0)
      {
        return false;
      }

      const double direction_norm = forward_direction.norm();
      if (direction_norm <= 1e-9)
      {
        return false;
      }
      const Eigen::Vector3d forward = forward_direction / direction_norm;
      const double candidate_progress = (candidate - start).dot(forward);
      const double minimum_progress =
          std::fmax(resolution, candidate_progress - resolution);
      const double maximum_distance =
          max_target_distance + resolution;
      const int maximum_shell =
          static_cast<int>(std::ceil(max_search_radius / resolution));

      for (int shell = 1; shell <= maximum_shell; ++shell)
      {
        bool found = false;
        double best_offset_sq = std::numeric_limits<double>::infinity();
        double best_progress = -std::numeric_limits<double>::infinity();

        for (int ix = -shell; ix <= shell; ++ix)
        {
          for (int iy = -shell; iy <= shell; ++iy)
          {
            for (int iz = -shell; iz <= shell; ++iz)
            {
              if (std::max(std::max(std::abs(ix), std::abs(iy)), std::abs(iz)) !=
                  shell)
              {
                continue;
              }

              const Eigen::Vector3d offset(
                  ix * resolution, iy * resolution, iz * resolution);
              const double offset_sq = offset.squaredNorm();
              if (offset_sq >
                  max_search_radius * max_search_radius + 1e-9)
              {
                continue;
              }

              const Eigen::Vector3d sample = candidate + offset;
              const Eigen::Vector3d from_start = sample - start;
              const double progress = from_start.dot(forward);
              // An obstacle-avoidance fallback must not trade horizontal
              // clearance for ground clearance.  Descending here can put the
              // vehicle inside the inflated virtual ground before the next
              // replan; search laterally or upward instead.
              if (sample.z() < candidate.z() - 1e-9 ||
                  progress < minimum_progress ||
                  from_start.norm() > maximum_distance ||
                  is_occupied(sample))
              {
                continue;
              }

              const bool ranks_better =
                  !found || offset_sq < best_offset_sq - 1e-9 ||
                  (std::abs(offset_sq - best_offset_sq) <= 1e-9 &&
                   progress > best_progress);
              if (!ranks_better ||
                  !isStraightLineFree(
                      start, sample, 0.5 * resolution, is_occupied,
                      is_start_voxel))
              {
                continue;
              }

              found = true;
              best_offset_sq = offset_sq;
              best_progress = progress;
              free_position = sample;
            }
          }
        }

        if (found)
        {
          return true;
        }
      }

      return false;
    }

    template <typename IsOccupied>
    bool findNearbyFreePosition(const Eigen::Vector3d &candidate,
                                const Eigen::Vector3d &start,
                                const Eigen::Vector3d &forward_direction,
                                const double resolution,
                                const double max_search_radius,
                                const double max_target_distance,
                                IsOccupied is_occupied,
                                Eigen::Vector3d &free_position)
    {
      return findNearbyFreePosition(
          candidate, start, forward_direction, resolution, max_search_radius,
          max_target_distance, is_occupied,
          [&](const Eigen::Vector3d &position) {
            return (position - start).squaredNorm() <= 1e-18;
          },
          free_position);
    }
  } // namespace planning_recovery_utils
} // namespace diff_planner

#endif // PLAN_MANAGE_PLANNING_RECOVERY_UTILS_H_
