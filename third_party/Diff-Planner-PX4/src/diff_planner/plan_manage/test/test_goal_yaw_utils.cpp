#include <gtest/gtest.h>

#include <plan_manage/goal_yaw_utils.h>

#include <cmath>
#include <limits>

namespace
{
  geometry_msgs::Quaternion quaternionFromRpy(double roll, double pitch, double yaw)
  {
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);

    geometry_msgs::Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return q;
  }
}

TEST(GoalYawUtils, ExtractsYawFromNormalizedAndScaledQuaternion)
{
  double yaw = 0.0;
  geometry_msgs::Quaternion q = quaternionFromRpy(0.2, -0.3, M_PI / 2.0);
  q.x *= 3.0;
  q.y *= 3.0;
  q.z *= 3.0;
  q.w *= 3.0;

  ASSERT_TRUE(diff_planner::goal_yaw_utils::quaternionToYaw(q, yaw));
  EXPECT_NEAR(yaw, M_PI / 2.0, 1e-9);
}

TEST(GoalYawUtils, TreatsZeroQuaternionAsUnspecifiedAndRejectsNonFiniteQuaternion)
{
  double yaw = 0.0;
  geometry_msgs::Quaternion q;
  // The goal CLI uses the all-zero quaternion when YAW_DEG is omitted.
  EXPECT_FALSE(diff_planner::goal_yaw_utils::quaternionToYaw(q, yaw));

  q.w = 1.0;
  q.z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(diff_planner::goal_yaw_utils::quaternionToYaw(q, yaw));

  q.x = std::numeric_limits<double>::max();
  q.y = std::numeric_limits<double>::max();
  q.z = 0.0;
  q.w = 0.0;
  EXPECT_FALSE(diff_planner::goal_yaw_utils::quaternionToYaw(q, yaw));
}

TEST(GoalYawUtils, WrapsAcrossPiUsingShortestAngularDistance)
{
  const double from = 179.0 * M_PI / 180.0;
  const double to = -179.0 * M_PI / 180.0;
  const double error = diff_planner::goal_yaw_utils::wrapAngle(to - from);
  EXPECT_NEAR(error, 2.0 * M_PI / 180.0, 1e-12);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
