#include <gtest/gtest.h>

#include <path_searching/endpoint_adjustment_utils.h>

namespace endpoint_adjustment = path_searching::endpoint_adjustment;

TEST(EndpointAdjustmentUtils, MovesAnOccupiedEndpointOutOfTheCollisionSegment)
{
  Eigen::Vector3d adjusted;
  ASSERT_TRUE(endpoint_adjustment::moveAwayFromOccupiedSegment(
      Eigen::Vector3d(3.0, 0.0, 1.0),
      Eigen::Vector3d(0.0, 0.0, 1.0),
      0.1, 0.6,
      [](const Eigen::Vector3d &point) {
        return point.x() < 3.3 ? 1 : 0;
      },
      adjusted));

  EXPECT_GT(adjusted.x(), 3.0);
  EXPECT_GE(adjusted.x(), 3.3);
  EXPECT_DOUBLE_EQ(adjusted.y(), 0.0);
  EXPECT_DOUBLE_EQ(adjusted.z(), 1.0);
}

TEST(EndpointAdjustmentUtils, MovesBothEndsAwayFromTheSameCollisionSegment)
{
  Eigen::Vector3d adjusted;
  ASSERT_TRUE(endpoint_adjustment::moveAwayFromOccupiedSegment(
      Eigen::Vector3d(0.0, 0.0, 1.0),
      Eigen::Vector3d(3.0, 0.0, 1.0),
      0.1, 0.6,
      [](const Eigen::Vector3d &point) {
        return point.x() > -0.3 ? 1 : 0;
      },
      adjusted));

  EXPECT_LT(adjusted.x(), 0.0);
  EXPECT_LE(adjusted.x(), -0.3);
}

TEST(EndpointAdjustmentUtils, NeverSearchesThroughTheObstacleInterior)
{
  Eigen::Vector3d adjusted;
  EXPECT_FALSE(endpoint_adjustment::moveAwayFromOccupiedSegment(
      Eigen::Vector3d(3.0, 0.0, 1.0),
      Eigen::Vector3d(0.0, 0.0, 1.0),
      0.1, 0.6,
      [](const Eigen::Vector3d &point) {
        return point.x() > 2.7 ? 1 : 0;
      },
      adjusted));
}

TEST(EndpointAdjustmentUtils, StopsAtTheConfiguredDistanceOrMapBoundary)
{
  Eigen::Vector3d adjusted;
  EXPECT_FALSE(endpoint_adjustment::moveAwayFromOccupiedSegment(
      Eigen::Vector3d::Zero(),
      -Eigen::Vector3d::UnitX(),
      0.1, 0.2,
      [](const Eigen::Vector3d &point) {
        return point.x() >= 0.3 ? 0 : 1;
      },
      adjusted));
  EXPECT_FALSE(endpoint_adjustment::moveAwayFromOccupiedSegment(
      Eigen::Vector3d::Zero(),
      -Eigen::Vector3d::UnitX(),
      0.1, 1.0,
      [](const Eigen::Vector3d &point) {
        return point.x() >= 0.2 ? -1 : 1;
      },
      adjusted));
}

TEST(EndpointAdjustmentUtils, RejectsInvalidSearchGeometry)
{
  Eigen::Vector3d adjusted;
  EXPECT_FALSE(endpoint_adjustment::moveAwayFromOccupiedSegment(
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),
      0.1, 1.0,
      [](const Eigen::Vector3d &) { return 0; },
      adjusted));
  EXPECT_FALSE(endpoint_adjustment::moveAwayFromOccupiedSegment(
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::UnitX(),
      0.0, 1.0,
      [](const Eigen::Vector3d &) { return 0; },
      adjusted));
  EXPECT_FALSE(endpoint_adjustment::moveAwayFromOccupiedSegment(
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::UnitX(),
      0.1, 0.0,
      [](const Eigen::Vector3d &) { return 0; },
      adjusted));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
