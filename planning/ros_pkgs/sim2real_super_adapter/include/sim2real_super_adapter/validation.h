#pragma once

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <sim2real_planning_msgs/PlannerGoal.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

namespace sim2real_super_adapter {

inline bool finite(double value) { return std::isfinite(value); }

inline bool finitePoint(const geometry_msgs::Point &point) {
  return finite(point.x) && finite(point.y) && finite(point.z);
}

inline bool finiteVector(const geometry_msgs::Vector3 &vector) {
  return finite(vector.x) && finite(vector.y) && finite(vector.z);
}

inline double quaternionNormSquared(const geometry_msgs::Quaternion &q) {
  return q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
}

inline bool zeroQuaternion(const geometry_msgs::Quaternion &q) {
  return quaternionNormSquared(q) <= 1.0e-12;
}

inline bool finiteGoalPose(const geometry_msgs::PoseStamped &goal,
                           bool constrain_yaw, std::string *reason) {
  if (goal.header.frame_id != "world") {
    if (reason != nullptr)
      *reason = "goal frame must be 'world'";
    return false;
  }
  if (goal.header.stamp.isZero()) {
    if (reason != nullptr)
      *reason = "goal stamp must be non-zero";
    return false;
  }
  if (!finitePoint(goal.pose.position)) {
    if (reason != nullptr)
      *reason = "goal position is non-finite";
    return false;
  }
  const auto &q = goal.pose.orientation;
  if (!finite(q.x) || !finite(q.y) || !finite(q.z) || !finite(q.w)) {
    if (reason != nullptr)
      *reason = "goal orientation is non-finite";
    return false;
  }
  const double norm_squared = quaternionNormSquared(q);
  if (!constrain_yaw) {
    if (norm_squared > 1.0e-12) {
      if (reason != nullptr) {
        *reason = "unconstrained-yaw goal must use the zero quaternion";
      }
      return false;
    }
    return true;
  }
  if (std::fabs(std::sqrt(norm_squared) - 1.0) > 1.0e-3 ||
      std::fabs(q.x) > 1.0e-3 || std::fabs(q.y) > 1.0e-3) {
    if (reason != nullptr) {
      *reason = "constrained goal must contain a unit yaw-only quaternion";
    }
    return false;
  }
  return true;
}

inline ros::Time
nativeGoalCorrelationStamp(const sim2real_planning_msgs::PlannerGoal &goal) {
  // The nested PoseStamped stamp belongs to the original user goal. The
  // enclosing PlannerGoal stamp is assigned later by the gateway.
  return goal.goal.header.stamp;
}

inline bool measurementStampIsCurrent(const ros::Time &stamp,
                                      const ros::Time &now, double timeout,
                                      const ros::Time &previous_stamp) {
  if (stamp.isZero() || now.isZero() || !finite(timeout) || timeout <= 0.0) {
    return false;
  }
  const double age = (now - stamp).toSec();
  if (!finite(age) || std::fabs(age) > timeout) {
    return false;
  }
  return previous_stamp.isZero() || stamp > previous_stamp;
}

enum class NativeTrajectoryIdDecision {
  ACCEPT_NEW,
  IGNORE_DUPLICATE,
  FAULT_BACKWARDS_OR_REPLAY,
};

inline NativeTrajectoryIdDecision
classifyNativeTrajectoryId(uint64_t candidate_id, uint64_t highest_id,
                           uint64_t accepted_current_id) {
  if (candidate_id == accepted_current_id && accepted_current_id != 0U) {
    return NativeTrajectoryIdDecision::IGNORE_DUPLICATE;
  }
  if (candidate_id <= highest_id) {
    return NativeTrajectoryIdDecision::FAULT_BACKWARDS_OR_REPLAY;
  }
  return NativeTrajectoryIdDecision::ACCEPT_NEW;
}

enum class NativeCommandDecision {
  NORMAL,
  BRAKE,
  REJECT_INVALID_OR_UNAUTHORIZED,
};

inline NativeCommandDecision
classifyNativeCommand(uint8_t trajectory_flag, bool goal_has_normal_command) {
  if (trajectory_flag == 1U) {
    return NativeCommandDecision::NORMAL;
  }
  if (trajectory_flag == 2U && goal_has_normal_command) {
    return NativeCommandDecision::BRAKE;
  }
  return NativeCommandDecision::REJECT_INVALID_OR_UNAUTHORIZED;
}

inline bool nativeOnlineGoalHandoffAllowed(
    bool has_goal, bool goal_dispatched, bool trajectory_committed,
    uint64_t accepted_native_trajectory_id, bool have_normal_command,
    bool latest_command_valid, bool native_finished, bool native_replan_hold,
    bool synthetic_close_goal, uint8_t current_progress) {
  constexpr uint8_t kFollowTrajectory = 4U;
  return has_goal && goal_dispatched && trajectory_committed &&
         accepted_native_trajectory_id > 0U && have_normal_command &&
         latest_command_valid && !native_finished && !native_replan_hold &&
         !synthetic_close_goal && current_progress == kFollowTrajectory;
}

inline bool nativeProgressIndicatesFinished(uint8_t previous_progress,
                                            uint8_t current_progress,
                                            bool trajectory_committed,
                                            bool have_normal_command,
                                            bool latest_command_valid) {
  constexpr uint8_t kFollowTrajectory = 4U;
  constexpr uint8_t kWaitGoal = 1U;
  return previous_progress == kFollowTrajectory &&
         current_progress == kWaitGoal && trajectory_committed &&
         have_normal_command && latest_command_valid;
}

inline bool nativeProgressStartsReplanHold(uint8_t previous_progress,
                                           uint8_t current_progress,
                                           bool trajectory_committed,
                                           bool have_normal_command,
                                           bool latest_command_valid) {
  constexpr uint8_t kGenerateTrajectory = 3U;
  constexpr uint8_t kFollowTrajectory = 4U;
  return previous_progress == kFollowTrajectory &&
         current_progress == kGenerateTrajectory && trajectory_committed &&
         have_normal_command && latest_command_valid;
}

inline bool nativeReplanHoldPermitsMissingCommand(bool replan_hold_active,
                                                  uint8_t current_progress) {
  constexpr uint8_t kGenerateTrajectory = 3U;
  return replan_hold_active && current_progress == kGenerateTrajectory;
}

inline bool nativeProgressFinishesReplanHold(uint8_t previous_progress,
                                             uint8_t current_progress,
                                             bool replan_hold_active) {
  constexpr uint8_t kWaitGoal = 1U;
  constexpr uint8_t kGenerateTrajectory = 3U;
  return replan_hold_active && previous_progress == kGenerateTrajectory &&
         current_progress == kWaitGoal;
}

inline bool nativeProgressFinishesCloseGoalWithoutTrajectory(
    uint8_t previous_progress, uint8_t current_progress, bool goal_dispatched,
    bool trajectory_committed) {
  constexpr uint8_t kWaitGoal = 1U;
  constexpr uint8_t kGenerateTrajectory = 3U;
  constexpr uint8_t kFollowTrajectory = 4U;
  return goal_dispatched && !trajectory_committed &&
         (previous_progress == kGenerateTrajectory ||
          previous_progress == kFollowTrajectory) &&
         current_progress == kWaitGoal;
}

inline uint64_t nextPublicTrajectoryId(uint64_t highest_public_id,
                                       uint64_t preferred_native_id) {
  if (highest_public_id == std::numeric_limits<uint64_t>::max()) {
    return 0U;
  }
  return std::max(highest_public_id + 1U, preferred_native_id);
}

inline bool nativeReplanHoldTimedOut(bool replan_hold_active,
                                     double elapsed_seconds,
                                     double planning_timeout_seconds) {
  return replan_hold_active && finite(elapsed_seconds) &&
         finite(planning_timeout_seconds) && planning_timeout_seconds > 0.0 &&
         elapsed_seconds > planning_timeout_seconds;
}

inline Eigen::Vector3d
bodyVectorToWorld(const Eigen::Quaterniond &body_in_world,
                  const Eigen::Vector3d &vector_in_body) {
  return body_in_world * vector_in_body;
}

inline bool
pointInsideBodyExclusionCylinder(const Eigen::Vector3d &point_in_world,
                                 const Eigen::Vector3d &body_position_in_world,
                                 const Eigen::Quaterniond &body_in_world,
                                 double radius, double min_z, double max_z) {
  const Eigen::Vector3d point_in_body =
      body_in_world.conjugate() * (point_in_world - body_position_in_world);
  return std::hypot(point_in_body.x(), point_in_body.y()) < radius &&
         point_in_body.z() >= min_z && point_in_body.z() <= max_z;
}

inline double yawFromQuaternion(const Eigen::Quaterniond &q) {
  const Eigen::Vector3d body_x = q.toRotationMatrix().col(0);
  return std::atan2(body_x.y(), body_x.x());
}

inline double wrapAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline bool
measuredStateSatisfiesGoal(const Eigen::Vector3d &position,
                           const Eigen::Vector3d &world_linear_velocity,
                           const Eigen::Quaterniond &body_in_world,
                           const Eigen::Vector3d &world_angular_velocity,
                           const geometry_msgs::PoseStamped &effective_goal,
                           bool constrain_yaw, double position_tolerance,
                           double velocity_tolerance, double yaw_tolerance,
                           double yaw_rate_tolerance) {
  const Eigen::Vector3d goal(effective_goal.pose.position.x,
                             effective_goal.pose.position.y,
                             effective_goal.pose.position.z);
  if ((position - goal).norm() > position_tolerance ||
      world_linear_velocity.norm() > velocity_tolerance ||
      std::fabs(world_angular_velocity.z()) > yaw_rate_tolerance) {
    return false;
  }
  if (!constrain_yaw)
    return true;
  const auto &goal_q = effective_goal.pose.orientation;
  const Eigen::Quaterniond goal_in_world(goal_q.w, goal_q.x, goal_q.y,
                                         goal_q.z);
  return std::fabs(wrapAngle(yawFromQuaternion(body_in_world) -
                             yawFromQuaternion(goal_in_world))) <=
         yaw_tolerance;
}

inline bool
measuredStateIsSettled(const Eigen::Vector3d &world_linear_velocity,
                       const Eigen::Vector3d &world_angular_velocity,
                       double velocity_tolerance,
                       double angular_velocity_tolerance) {
  return world_linear_velocity.norm() <= velocity_tolerance &&
         world_angular_velocity.norm() <= angular_velocity_tolerance;
}

} // namespace sim2real_super_adapter
