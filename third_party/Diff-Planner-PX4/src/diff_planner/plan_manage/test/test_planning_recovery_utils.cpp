#include <gtest/gtest.h>

#include <plan_manage/planning_recovery_utils.h>

#include <limits>

namespace recovery = diff_planner::planning_recovery_utils;

TEST(PlanningRecoveryUtils, AcceptsOnlyAnActiveFiniteLocalTrajectory)
{
  EXPECT_TRUE(recovery::isLocalTrajectoryUsable(12.0, 10.0, 3.0));
  EXPECT_FALSE(recovery::isLocalTrajectoryUsable(9.9, 10.0, 3.0));
  EXPECT_FALSE(recovery::isLocalTrajectoryUsable(13.0, 10.0, 3.0));
  EXPECT_FALSE(recovery::isLocalTrajectoryUsable(10.0, 10.0, 0.0));
  EXPECT_FALSE(recovery::isLocalTrajectoryUsable(
      std::numeric_limits<double>::quiet_NaN(), 10.0, 3.0));
}

TEST(PlanningRecoveryUtils, KeepsAFreeCandidateUnchanged)
{
  double free_time = -1.0;
  ASSERT_TRUE(recovery::findFreeTimeByBacktracking(
      2.0, 8.0, 0.5,
      [](const double) { return false; }, free_time));
  EXPECT_DOUBLE_EQ(free_time, 8.0);
}

TEST(PlanningRecoveryUtils, SelectsTheNearestEarlierFreeSample)
{
  double free_time = -1.0;
  ASSERT_TRUE(recovery::findFreeTimeByBacktracking(
      2.0, 8.0, 0.5,
      [](const double time) { return time >= 7.0; }, free_time));
  EXPECT_DOUBLE_EQ(free_time, 6.5);
}

TEST(PlanningRecoveryUtils, IncludesTheLowerBoundAndReportsNoFallback)
{
  double free_time = -1.0;
  ASSERT_TRUE(recovery::findFreeTimeByBacktracking(
      2.0, 2.8, 0.5,
      [](const double time) { return time > 2.0; }, free_time));
  EXPECT_DOUBLE_EQ(free_time, 2.0);

  EXPECT_FALSE(recovery::findFreeTimeByBacktracking(
      2.0, 8.0, 0.5,
      [](const double) { return true; }, free_time));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
