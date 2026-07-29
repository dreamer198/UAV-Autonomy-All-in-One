#include <gtest/gtest.h>
#include <sim2real_fast_adapter/ground_plane_filter.h>
#include <sim2real_fast_adapter/validation.h>
#include <sim2real_fast_adapter/visualization_map.h>

using sim2real_fast_adapter::bodyVelocityToWorld;
using sim2real_fast_adapter::buildVirtualFloor;
using sim2real_fast_adapter::finitePose;
using sim2real_fast_adapter::finiteTrajectorySetpoint;
using sim2real_fast_adapter::fitDominantGroundPlane;
using sim2real_fast_adapter::goalAboveVirtualFloor;
using sim2real_fast_adapter::goalInsideMapBounds;
using sim2real_fast_adapter::measurementStampIsCurrent;
using sim2real_fast_adapter::measuredStateSatisfiesGoal;
using sim2real_fast_adapter::pointInsideBodyExclusionCylinder;
using sim2real_fast_adapter::RealObstacleVisualizationMap;
using sim2real_fast_adapter::removeGroundPlanePoints;

TEST(FastAdapterValidation, RotatesBodyVelocityIntoWorld) {
  const Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
  const Eigen::Vector3d world = bodyVelocityToWorld(q, Eigen::Vector3d::UnitX());
  EXPECT_NEAR(world.x(), 0.0, 1e-12);
  EXPECT_NEAR(world.y(), 1.0, 1e-12);
  EXPECT_NEAR(world.z(), 0.0, 1e-12);
}

TEST(FastAdapterValidation, AcceptsFiniteSetpointAboveNominalOptimizerValues) {
  EXPECT_TRUE(finiteTrajectorySetpoint(
      Eigen::Vector3d(2.0, -1.0, 1.5),
      Eigen::Vector3d(3.0, -2.0, 1.0),
      Eigen::Vector3d(4.0, -3.0, 2.0), 0.5, 1.2));

  EXPECT_FALSE(finiteTrajectorySetpoint(
      Eigen::Vector3d(2.0, -1.0, 1.5),
      Eigen::Vector3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0),
      Eigen::Vector3d::Zero(), 0.0, 0.0));
}

TEST(FastAdapterValidation, RemovesObservedMid360AirframeReturns) {
  const Eigen::Vector3d body_position = Eigen::Vector3d::Zero();
  const Eigen::Quaterniond body_in_world = Eigen::Quaterniond::Identity();

  // These body-frame samples reproduce the closest points captured before
  // the three Fast-Topo false emergency stops. Their common z coordinate is
  // the simulated MID-360 mount plane.
  EXPECT_TRUE(pointInsideBodyExclusionCylinder(
      Eigen::Vector3d(0.1663, 0.2368, 0.0711), body_position,
      body_in_world, 0.35, -0.20, 0.20));
  EXPECT_TRUE(pointInsideBodyExclusionCylinder(
      Eigen::Vector3d(0.2125, -0.0515, 0.0721), body_position,
      body_in_world, 0.35, -0.20, 0.20));
  EXPECT_TRUE(pointInsideBodyExclusionCylinder(
      Eigen::Vector3d(0.1629, 0.2347, 0.0721), body_position,
      body_in_world, 0.35, -0.20, 0.20));
}

TEST(FastAdapterValidation, BodyExclusionTracksVehiclePose) {
  const Eigen::Vector3d body_position(4.0, -2.0, 1.0);
  const Eigen::Quaterniond body_in_world(
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()));
  const Eigen::Vector3d self_return_world =
      body_position +
      body_in_world * Eigen::Vector3d(0.20, 0.10, 0.07);
  EXPECT_TRUE(pointInsideBodyExclusionCylinder(
      self_return_world, body_position, body_in_world,
      0.35, -0.20, 0.20));

  EXPECT_FALSE(pointInsideBodyExclusionCylinder(
      body_position + body_in_world * Eigen::Vector3d(0.36, 0.0, 0.07),
      body_position, body_in_world, 0.35, -0.20, 0.20));
  EXPECT_FALSE(pointInsideBodyExclusionCylinder(
      body_position + body_in_world * Eigen::Vector3d(0.10, 0.0, 0.25),
      body_position, body_in_world, 0.35, -0.20, 0.20));
}

TEST(FastAdapterValidation, AppliesOnlyInflationMarginToFixedMap) {
  const Eigen::Vector3d origin(-15.0, -15.0, -1.0);
  const Eigen::Vector3d size(30.0, 30.0, 5.0);
  geometry_msgs::Point goal;
  goal.x = 14.9;
  goal.y = -14.9;
  goal.z = 1.0;
  std::string reason;
  EXPECT_TRUE(goalInsideMapBounds(goal, origin, size, 0.1, &reason)) << reason;
  goal.x = 15.0;
  EXPECT_FALSE(goalInsideMapBounds(goal, origin, size, 0.1, &reason));
}

TEST(FastAdapterValidation, UnifiedMapHasThirtyMeterHorizontalCoverage) {
  const Eigen::Vector3d origin(-15.0, -15.0, -1.0);
  const Eigen::Vector3d size(30.0, 30.0, 5.0);
  geometry_msgs::Point goal;
  goal.x = -14.9;
  goal.y = 14.9;
  goal.z = 1.0;
  std::string reason;
  EXPECT_TRUE(goalInsideMapBounds(goal, origin, size, 0.1, &reason)) << reason;
  goal.y = 15.0;
  EXPECT_FALSE(goalInsideMapBounds(goal, origin, size, 0.1, &reason));
}

TEST(FastAdapterValidation, UnifiedMapContainsIndoorMissionExtrema) {
  const Eigen::Vector3d origin(-15.0, -15.0, -1.0);
  const Eigen::Vector3d size(30.0, 30.0, 5.0);
  geometry_msgs::Point goal;
  std::string reason;
  goal.x = -0.0216;
  goal.y = -2.1467;
  goal.z = 1.0;
  EXPECT_TRUE(goalInsideMapBounds(goal, origin, size, 0.1, &reason)) << reason;
  goal.x = 3.7026;
  goal.y = 1.0273;
  EXPECT_TRUE(goalInsideMapBounds(goal, origin, size, 0.1, &reason)) << reason;
}

TEST(FastAdapterValidation, DefaultRvizHeightClearsPhysicalVirtualFloor) {
  std::string reason;
  EXPECT_TRUE(goalAboveVirtualFloor(
      1.0, 0.0, 0.3, 0.5, 0.1, &reason))
      << reason;
  EXPECT_FALSE(goalAboveVirtualFloor(
      1.0, 0.2, 0.3, 0.5, 0.1, &reason));
}

TEST(FastAdapterValidation, ConstrainedYawRequiresUnitQuaternion) {
  geometry_msgs::PoseStamped goal;
  goal.header.frame_id = "world";
  goal.pose.orientation.w = 2.0;
  std::string reason;
  EXPECT_FALSE(finitePose(goal, true, &reason));
  goal.pose.orientation.w = 1.0;
  EXPECT_TRUE(finitePose(goal, true, &reason));
}

TEST(FastAdapterValidation, RejectsStaleFutureAndReplayedMeasurements) {
  const ros::Time now(100.0);
  EXPECT_TRUE(measurementStampIsCurrent(ros::Time(99.8), now, 0.5, ros::Time()));
  EXPECT_FALSE(measurementStampIsCurrent(ros::Time(99.4), now, 0.5, ros::Time()));
  EXPECT_FALSE(measurementStampIsCurrent(ros::Time(100.2), now, 0.5, ros::Time()));
  EXPECT_FALSE(
      measurementStampIsCurrent(ros::Time(99.8), now, 0.5, ros::Time(99.8)));
}

TEST(FastAdapterValidation, RequiresMeasuredPositionAndVelocityForReached) {
  geometry_msgs::PoseStamped goal;
  goal.header.frame_id = "world";
  goal.pose.position.x = 6.0;
  goal.pose.position.y = -2.0;
  goal.pose.position.z = 1.0;
  const Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  const Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();

  EXPECT_FALSE(measuredStateSatisfiesGoal(
      Eigen::Vector3d(2.0, -2.0, 0.0), Eigen::Vector3d::Zero(),
      orientation, angular_velocity, goal, false, 0.35, 0.2,
      5.0 * M_PI / 180.0, 10.0 * M_PI / 180.0));
  EXPECT_FALSE(measuredStateSatisfiesGoal(
      Eigen::Vector3d(6.0, -2.0, 1.0), Eigen::Vector3d(0.21, 0.0, 0.0),
      orientation, angular_velocity, goal, false, 0.35, 0.2,
      5.0 * M_PI / 180.0, 10.0 * M_PI / 180.0));
  EXPECT_TRUE(measuredStateSatisfiesGoal(
      Eigen::Vector3d(6.1, -2.0, 1.0), Eigen::Vector3d(0.1, 0.0, 0.0),
      orientation, angular_velocity, goal, false, 0.35, 0.2,
      5.0 * M_PI / 180.0, 10.0 * M_PI / 180.0));
}

TEST(FastAdapterValidation, EnforcesYawOnlyForConstrainedGoals) {
  geometry_msgs::PoseStamped goal;
  goal.header.frame_id = "world";
  goal.pose.position.z = 1.0;
  goal.pose.orientation.w = 1.0;
  const Eigen::Vector3d position(0.0, 0.0, 1.0);
  const Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  const Eigen::Quaterniond ten_degree_yaw(
      Eigen::AngleAxisd(10.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()));

  EXPECT_FALSE(measuredStateSatisfiesGoal(
      position, velocity, ten_degree_yaw, Eigen::Vector3d::Zero(),
      goal, true, 0.35, 0.2, 5.0 * M_PI / 180.0,
      10.0 * M_PI / 180.0));
  EXPECT_TRUE(measuredStateSatisfiesGoal(
      position, velocity, ten_degree_yaw, Eigen::Vector3d::Zero(),
      goal, false, 0.35, 0.2, 5.0 * M_PI / 180.0,
      10.0 * M_PI / 180.0));
  EXPECT_FALSE(measuredStateSatisfiesGoal(
      position, velocity, Eigen::Quaterniond::Identity(),
      Eigen::Vector3d(0.0, 0.0, 11.0 * M_PI / 180.0),
      goal, true, 0.35, 0.2, 5.0 * M_PI / 180.0,
      10.0 * M_PI / 180.0));
}

TEST(FastAdapterValidation, BuildsDenseAdapterPrivateVirtualFloor) {
  std::vector<Eigen::Vector3d> points;
  std::string reason;
  ASSERT_TRUE(buildVirtualFloor(
      Eigen::Vector3d(0.0, 0.0, 1.5),
      Eigen::Vector3d(-15.0, -15.0, -1.0),
      Eigen::Vector3d(30.0, 30.0, 5.0),
      Eigen::Vector3d(5.5, 5.5, 4.5), 0.1, 0.0, &points, &reason))
      << reason;
  EXPECT_GT(points.size(), 10000U);
  for (const Eigen::Vector3d& point : points) {
    EXPECT_NEAR(point.z(), 0.0, 1e-12);
    EXPECT_LT(std::abs(point.x()), 5.5);
    EXPECT_LT(std::abs(point.y()), 5.5);
  }
}

TEST(FastAdapterValidation, RejectsVirtualFloorOutsideVerticalUpdateWindow) {
  std::vector<Eigen::Vector3d> points;
  std::string reason;
  EXPECT_FALSE(buildVirtualFloor(
      Eigen::Vector3d(0.0, 0.0, 3.8),
      Eigen::Vector3d(-15.0, -15.0, -1.0),
      Eigen::Vector3d(30.0, 30.0, 5.0),
      Eigen::Vector3d(5.5, 5.5, 1.0), 0.1, 0.2, &points, &reason));
}

TEST(FastAdapterValidation, VisualizesLowRealObstaclesWithoutHeightFiltering) {
  RealObstacleVisualizationMap map(
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(2.0, 2.0, 2.0),
      Eigen::Vector3d(1.0, 1.0, 1.0),
      0.1, 0.1, 0.3, true);
  std::string reason;
  ASSERT_TRUE(map.update(
      {Eigen::Vector3d(0.25, 0.25, 0.15)},
      Eigen::Vector3d(0.5, 0.5, 0.5), &reason))
      << reason;

  const std::vector<Eigen::Vector3d> occupied =
      map.occupancyPoints(Eigen::Vector3d(0.5, 0.5, 0.5));
  ASSERT_EQ(occupied.size(), 1U);
  EXPECT_NEAR(occupied.front().z(), 0.15, 1e-12);

  const std::vector<Eigen::Vector3d> inflated =
      map.inflatedOccupancyPoints(Eigen::Vector3d(0.5, 0.5, 0.5));
  EXPECT_FALSE(inflated.empty());
  EXPECT_TRUE(std::any_of(
      inflated.begin(), inflated.end(),
      [](const Eigen::Vector3d& point) { return point.z() < 0.36; }));
}

TEST(FastAdapterValidation, QueriesObservedInflatedObstacleClearance) {
  RealObstacleVisualizationMap map(
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(2.0, 2.0, 2.0),
      Eigen::Vector3d(1.0, 1.0, 1.0),
      0.1, 0.0, 0.0, true);
  std::string reason;
  ASSERT_TRUE(map.update(
      {Eigen::Vector3d(0.55, 0.55, 0.55)},
      Eigen::Vector3d(0.5, 0.5, 0.5), &reason))
      << reason;

  double nearest = 0.0;
  EXPECT_TRUE(map.inflatedObstacleWithin(
      Eigen::Vector3d(0.75, 0.55, 0.55), 0.21, &nearest));
  EXPECT_NEAR(nearest, 0.20, 1e-12);
  EXPECT_FALSE(map.inflatedObstacleWithin(
      Eigen::Vector3d(0.95, 0.55, 0.55), 0.30, &nearest));
  EXPECT_NEAR(nearest, 0.40, 1e-12);
}

TEST(FastAdapterValidation, VisualizationMapAccumulatesOnlySuppliedRealPoints) {
  RealObstacleVisualizationMap map(
      Eigen::Vector3d(-1.0, -1.0, -1.0),
      Eigen::Vector3d(2.0, 2.0, 2.0),
      Eigen::Vector3d(1.0, 1.0, 1.0),
      0.1, 0.0, 0.0, true);
  const Eigen::Vector3d sensor_position = Eigen::Vector3d::Zero();
  std::string reason;
  ASSERT_TRUE(map.update(
      {Eigen::Vector3d(-0.25, 0.05, 0.05)}, sensor_position, &reason))
      << reason;
  ASSERT_TRUE(map.update(
      {Eigen::Vector3d(0.25, 0.05, 0.05)}, sensor_position, &reason))
      << reason;

  const std::vector<Eigen::Vector3d> occupied =
      map.occupancyPoints(sensor_position);
  ASSERT_EQ(occupied.size(), 2U);
  for (const Eigen::Vector3d& point : occupied) {
    // No synthetic plane appears: every displayed voxel is one of the two
    // sensor-supplied obstacle voxels.
    EXPECT_NEAR(point.y(), 0.05, 1e-12);
    EXPECT_NEAR(point.z(), 0.05, 1e-12);
  }
}

TEST(FastAdapterValidation, RemovesOnlyTheObservedGroundPlane) {
  std::vector<Eigen::Vector3d> points;
  for (int x = -20; x <= 20; ++x) {
    for (int y = -20; y <= 20; ++y) {
      const double px = 0.1 * static_cast<double>(x);
      const double py = 0.1 * static_cast<double>(y);
      const double pz = 0.01 * px - 0.005 * py - 0.15;
      points.emplace_back(px, py, pz);
    }
  }
  // A five-centimetre obstacle remains visible even though it is below an
  // absolute z cutoff such as the former 0.36 m display threshold.
  points.emplace_back(0.5, 0.5, -0.0975);
  points.emplace_back(0.5, 0.5, -0.0475);

  Eigen::Vector4d plane;
  ASSERT_TRUE(fitDominantGroundPlane(
      points, -0.6, 0.3, 0.02, 15.0 * M_PI / 180.0,
      200U, 2.0, 96, &plane));
  size_t removed = 0U;
  const std::vector<Eigen::Vector3d> visible =
      removeGroundPlanePoints(points, plane, 0.02, &removed);

  EXPECT_GE(removed, 1600U);
  ASSERT_EQ(visible.size(), 2U);
  EXPECT_NEAR(visible[0].z(), -0.0975, 1e-12);
  EXPECT_NEAR(visible[1].z(), -0.0475, 1e-12);
}

TEST(FastAdapterValidation, DoesNotClassifyANarrowObjectAsGround) {
  std::vector<Eigen::Vector3d> wall;
  for (int x = -20; x <= 20; ++x) {
    for (int z = 0; z <= 20; ++z) {
      wall.emplace_back(0.1 * static_cast<double>(x), 0.0,
                        0.02 * static_cast<double>(z) - 0.2);
    }
  }
  Eigen::Vector4d plane;
  EXPECT_FALSE(fitDominantGroundPlane(
      wall, -0.6, 0.3, 0.02, 15.0 * M_PI / 180.0,
      200U, 2.0, 96, &plane));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
