#ifndef PLAN_MANAGE_GOAL_YAW_UTILS_H_
#define PLAN_MANAGE_GOAL_YAW_UTILS_H_

#include <geometry_msgs/Quaternion.h>

#include <cmath>

namespace diff_planner
{
  namespace goal_yaw_utils
  {
    inline double wrapAngle(double angle)
    {
      return std::atan2(std::sin(angle), std::cos(angle));
    }

    inline bool quaternionToYaw(const geometry_msgs::Quaternion &q_msg, double &yaw)
    {
      if (!std::isfinite(q_msg.x) || !std::isfinite(q_msg.y) ||
          !std::isfinite(q_msg.z) || !std::isfinite(q_msg.w))
      {
        return false;
      }

      const double norm = std::sqrt(q_msg.x * q_msg.x + q_msg.y * q_msg.y +
                                    q_msg.z * q_msg.z + q_msg.w * q_msg.w);
      if (!std::isfinite(norm) || norm < 1e-6)
      {
        return false;
      }

      const double x = q_msg.x / norm;
      const double y = q_msg.y / norm;
      const double z = q_msg.z / norm;
      const double w = q_msg.w / norm;
      yaw = std::atan2(2.0 * (w * z + x * y),
                       1.0 - 2.0 * (y * y + z * z));
      return std::isfinite(yaw);
    }
  } // namespace goal_yaw_utils
} // namespace diff_planner

#endif // PLAN_MANAGE_GOAL_YAW_UTILS_H_
