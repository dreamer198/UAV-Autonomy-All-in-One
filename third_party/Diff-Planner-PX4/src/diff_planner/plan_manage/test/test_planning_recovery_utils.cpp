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

TEST(PlanningRecoveryUtils, ReplanFailureWindowUsesElapsedTimeAndRetrySpacing)
{
  recovery::ReplanFailureWindow window;
  window.configure(0.1, 1.0);
  window.reset(10.0);

  EXPECT_TRUE(window.shouldAttempt(10.0));
  window.recordFailure(10.0);
  EXPECT_TRUE(window.active());
  EXPECT_FALSE(window.shouldAttempt(10.05));
  EXPECT_TRUE(window.shouldAttempt(10.1));
  EXPECT_FALSE(window.timedOut(10.12));
  EXPECT_FALSE(window.timedOut(10.99));
  EXPECT_TRUE(window.timedOut(11.0));
}

TEST(PlanningRecoveryUtils, ReplanFailureWindowResetsAfterSuccessOrClockRewind)
{
  recovery::ReplanFailureWindow window;
  window.configure(0.1, 1.0);
  window.reset(20.0);
  window.recordFailure(20.0);

  window.reset(20.2);
  EXPECT_FALSE(window.active());
  EXPECT_TRUE(window.shouldAttempt(20.2));
  EXPECT_FALSE(window.timedOut(30.0));

  window.recordFailure(30.0);
  EXPECT_TRUE(window.shouldAttempt(5.0));
  EXPECT_FALSE(window.active());
  EXPECT_FALSE(window.timedOut(7.1));
}

TEST(PlanningRecoveryUtils, StuckMonitorAllowsLateralObstacleDetour)
{
  recovery::StuckProgressMonitor monitor;
  monitor.configure(0.1, 5.0);
  monitor.reset(0.0);

  EXPECT_FALSE(monitor.updateVehicleMotion(
      0.1, Eigen::Vector3d(0.0, 0.0, 1.5)));
  EXPECT_FALSE(monitor.updateVehicleMotion(
      4.0, Eigen::Vector3d(0.0, 0.11, 1.5)));
  EXPECT_FALSE(monitor.updateVehicleMotion(
      8.9, Eigen::Vector3d(0.0, 0.22, 1.5)));
}

TEST(PlanningRecoveryUtils, StuckMonitorAllowsBackwardObstacleDetour)
{
  recovery::StuckProgressMonitor monitor;
  monitor.configure(0.1, 5.0);
  monitor.reset(10.0);

  EXPECT_FALSE(monitor.updateVehicleMotion(
      10.1, Eigen::Vector3d(2.5, 0.0, 1.5)));
  EXPECT_FALSE(monitor.updateVehicleMotion(
      14.0, Eigen::Vector3d(2.62, 0.0, 1.5)));
  EXPECT_FALSE(monitor.updateVehicleMotion(
      18.9, Eigen::Vector3d(2.74, 0.0, 1.5)));
}

TEST(PlanningRecoveryUtils, StuckMonitorReportsNoProgressAfterTimeout)
{
  recovery::StuckProgressMonitor monitor;
  monitor.configure(0.1, 5.0);
  monitor.reset(20.0);

  const Eigen::Vector3d start(1.0, 2.0, 1.5);
  EXPECT_FALSE(monitor.updateVehicleMotion(20.1, start));
  EXPECT_FALSE(monitor.updateVehicleMotion(25.0, start));
  EXPECT_TRUE(monitor.updateVehicleMotion(25.2, start));
}

TEST(PlanningRecoveryUtils, MovingLocalTargetDoesNotHideStationaryVehicle)
{
  recovery::StuckProgressMonitor monitor;
  monitor.configure(0.1, 5.0);
  monitor.reset(0.0);
  const Eigen::Vector3d vehicle(0.0, 0.0, 1.5);
  Eigen::Vector3d local_target(1.0, 0.0, 1.5);

  EXPECT_FALSE(monitor.updateVehicleMotion(0.1, vehicle));
  local_target.x() += 1.0;
  EXPECT_FALSE(monitor.updateVehicleMotion(4.9, vehicle));
  local_target.x() += 1.0;
  EXPECT_TRUE(monitor.updateVehicleMotion(5.2, vehicle));
  EXPECT_DOUBLE_EQ(local_target.x(), 3.0);
}

TEST(PlanningRecoveryUtils, RequiresInflationClearanceFromVerticalFences)
{
  EXPECT_TRUE(recovery::isWithinVerticalClearance(1.0, 0.1, 3.0, 0.33));
  EXPECT_FALSE(recovery::isWithinVerticalClearance(0.43, 0.1, 3.0, 0.33));
  EXPECT_FALSE(recovery::isWithinVerticalClearance(2.67, 0.1, 3.0, 0.33));
  EXPECT_FALSE(recovery::isWithinVerticalClearance(
      std::numeric_limits<double>::quiet_NaN(), 0.1, 3.0, 0.33));
}

TEST(PlanningRecoveryUtils, BoundsGlobalTrajectoryVisualizationSamples)
{
  EXPECT_EQ(
      recovery::boundedTrajectorySampleCount(0.25, 0.1, 5000), 4u);
  EXPECT_EQ(
      recovery::boundedTrajectorySampleCount(1.0e50, 0.1, 5000), 5000u);
  EXPECT_EQ(
      recovery::boundedTrajectorySampleCount(
          std::numeric_limits<double>::infinity(), 0.1, 5000),
      0u);
  EXPECT_FALSE(recovery::isNumericallySafeQuinticDuration(1.0e61));
}

TEST(PlanningRecoveryUtils, FindsExtremeHorizonCrossingWithBoundedWork)
{
  constexpr double duration = 1.0e24;
  constexpr double distance = 1.0e24;
  std::size_t evaluations = 0;
  const auto position_at_time = [&](const double time) {
    ++evaluations;
    const double ratio = time / duration;
    const double progress =
        10.0 * ratio * ratio * ratio -
        15.0 * ratio * ratio * ratio * ratio +
        6.0 * ratio * ratio * ratio * ratio * ratio;
    return Eigen::Vector3d(distance * progress, 0.0, 0.0);
  };

  double crossing_time = 0.0;
  ASSERT_TRUE(recovery::findMonotonicHorizonCrossingTime(
      0.0, duration, 7.5, Eigen::Vector3d::Zero(), position_at_time,
      crossing_time));
  EXPECT_LE(evaluations, 98u);
  const double reached_distance = position_at_time(crossing_time).x();
  EXPECT_GE(reached_distance, 7.5);
  EXPECT_LT(reached_distance, 7.5001);
}

TEST(PlanningRecoveryUtils, ReportsWhenGoalRemainsInsideHorizon)
{
  double crossing_time = 0.0;
  EXPECT_FALSE(recovery::findMonotonicHorizonCrossingTime(
      0.0, 10.0, 7.5, Eigen::Vector3d::Zero(),
      [](const double time) {
        return Eigen::Vector3d(0.5 * time, 0.0, 0.0);
      },
      crossing_time));
}

TEST(PlanningRecoveryUtils, AllowsAPathToLeaveItsCurrentOccupiedVoxel)
{
  const Eigen::Vector3d start(0.01, 0.0, 0.0);
  const Eigen::Vector3d end(0.31, 0.0, 0.0);
  EXPECT_TRUE(recovery::isStraightLineFree(
      start, end, 0.05,
      [](const Eigen::Vector3d &position) {
        return position.x() < 0.1;
      },
      [](const Eigen::Vector3d &position) {
        return position.x() < 0.1;
      }));
}

TEST(PlanningRecoveryUtils, StillRejectsAnObstacleBeyondTheStartVoxel)
{
  const Eigen::Vector3d start(0.01, 0.0, 0.0);
  const Eigen::Vector3d end(0.31, 0.0, 0.0);
  EXPECT_FALSE(recovery::isStraightLineFree(
      start, end, 0.05,
      [](const Eigen::Vector3d &position) {
        return position.x() < 0.2;
      },
      [](const Eigen::Vector3d &position) {
        return position.x() < 0.1;
      }));
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

TEST(PlanningRecoveryUtils, BoundsExtremeOccupiedTargetBacktracking)
{
  constexpr std::size_t max_evaluations = 32;
  std::size_t evaluations = 0;
  double free_time = -1.0;

  ASSERT_TRUE(recovery::findFreeTimeByBacktracking(
      0.0, 1.0e50, 0.1,
      [&](const double time) {
        ++evaluations;
        return time > 0.0;
      },
      free_time, max_evaluations));

  EXPECT_DOUBLE_EQ(free_time, 0.0);
  EXPECT_EQ(evaluations, max_evaluations);
}

TEST(PlanningRecoveryUtils, RejectsInvalidBacktrackingEvaluationLimit)
{
  double free_time = -1.0;
  EXPECT_FALSE(recovery::findFreeTimeByBacktracking(
      0.0, 1.0, 0.1,
      [](const double) { return false; }, free_time, 1));
}

TEST(PlanningRecoveryUtils, FindsAForwardFreePointBesideAnOccupiedTarget)
{
  Eigen::Vector3d free_position;
  ASSERT_TRUE(recovery::findNearbyFreePosition(
      Eigen::Vector3d(3.0, 0.0, 0.5),
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::UnitX(),
      0.1, 0.8, 3.0,
      [](const Eigen::Vector3d &position) {
        return std::abs(position.x() - 3.0) <= 0.2 &&
               std::abs(position.y()) <= 0.2 &&
               std::abs(position.z() - 0.5) <= 0.2;
      },
      free_position));

  EXPECT_GT(free_position.x(), 0.0);
  EXPECT_LE(free_position.norm(), 3.1);
  EXPECT_GE(free_position.z(), 0.5);
  EXPECT_TRUE(
      std::abs(free_position.y()) > 0.2 ||
      std::abs(free_position.z() - 0.5) > 0.2);
}

TEST(PlanningRecoveryUtils, RejectsFreeFallbacksBehindAnOccupiedWall)
{
  Eigen::Vector3d free_position;
  EXPECT_FALSE(recovery::findNearbyFreePosition(
      Eigen::Vector3d(3.0, 0.0, 0.5),
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::UnitX(),
      0.1, 0.3, 3.0,
      [](const Eigen::Vector3d &position) {
        return std::abs(position.x() - 1.5) <= 0.15;
      },
      free_position));
}

TEST(PlanningRecoveryUtils, NeverUsesAFreePointBelowTheCandidate)
{
  Eigen::Vector3d free_position;
  EXPECT_FALSE(recovery::findNearbyFreePosition(
      Eigen::Vector3d(3.0, 0.0, 0.5),
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::UnitX(),
      0.1, 0.3, 3.0,
      [](const Eigen::Vector3d &position) {
        return position.z() >= 0.5 - 1e-9;
      },
      free_position));
}

TEST(PlanningRecoveryUtils, RejectsFreePointsWithoutForwardProgress)
{
  Eigen::Vector3d free_position;
  EXPECT_FALSE(recovery::findNearbyFreePosition(
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::UnitX(),
      0.1, 0.2, 0.2,
      [](const Eigen::Vector3d &position) {
        return position.x() >= 0.1;
      },
      free_position));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
