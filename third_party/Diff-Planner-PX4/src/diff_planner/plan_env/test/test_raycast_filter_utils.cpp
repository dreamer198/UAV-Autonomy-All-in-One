#include <gtest/gtest.h>

#include <limits>

#include <plan_env/raycast_filter_utils.h>

namespace filter = plan_env::raycast_filter;

TEST(RaycastFilterUtils, ValidatesMinimumRayLength)
{
  EXPECT_TRUE(filter::isValidMinimumRayLength(0.0));
  EXPECT_TRUE(filter::isValidMinimumRayLength(0.1));
  EXPECT_FALSE(filter::isValidMinimumRayLength(-0.1));
  EXPECT_FALSE(filter::isValidMinimumRayLength(
      std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(filter::isValidMinimumRayLength(
      std::numeric_limits<double>::quiet_NaN()));
}

TEST(RaycastFilterUtils, RejectsEndpointsInsideMinimumRange)
{
  const Eigen::Vector3d camera(1.0, 2.0, 3.0);
  EXPECT_FALSE(filter::shouldProcessRay(
      camera, camera + Eigen::Vector3d(0.05, 0.0, 0.0), 0.1));
  EXPECT_TRUE(filter::shouldProcessRay(
      camera, camera + Eigen::Vector3d(0.1, 0.0, 0.0), 0.1));
  EXPECT_TRUE(filter::shouldProcessRay(
      camera, camera + Eigen::Vector3d(0.1, 0.1, 0.1), 0.1));
}

TEST(RaycastFilterUtils, RejectsNonFiniteRayGeometry)
{
  Eigen::Vector3d invalid_endpoint = Eigen::Vector3d::Zero();
  invalid_endpoint.z() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(filter::shouldProcessRay(
      Eigen::Vector3d::Zero(), invalid_endpoint, 0.1));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
