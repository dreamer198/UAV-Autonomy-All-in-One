#include <gtest/gtest.h>

#include <limits>

#include <boost/make_shared.hpp>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <se3_controller/se3_controller.hpp>

namespace
{
nav_msgs::OdometryPtr validOdom(const double stamp, const double x = 0.0)
{
  nav_msgs::OdometryPtr msg = boost::make_shared<nav_msgs::Odometry>();
  msg->header.stamp = ros::Time(stamp);
  msg->pose.pose.position.x = x;
  msg->pose.pose.orientation.w = 1.0;
  return msg;
}

sensor_msgs::ImuPtr validImu(const double stamp)
{
  sensor_msgs::ImuPtr msg = boost::make_shared<sensor_msgs::Imu>();
  msg->header.stamp = ros::Time(stamp);
  msg->orientation.w = 1.0;
  return msg;
}

void configureDerivativeController(SE3_CONTROLLER &controller)
{
  controller.init(0.5, 0.8, 0.0, 0.9, true, false);
  ASSERT_TRUE(controller.setup(
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),
      100.0, 100.0, 100.0,
      1000.0, 1000.0, 1000.0));
  controller.setIntegral(Eigen::Vector3d::Zero(), 0.0);
}

Controller_Output_t runDerivativeStep(const double dt)
{
  SE3_CONTROLLER controller;
  configureDerivativeController(controller);
  Desired_State_t desired;
  Controller_Output_t output;
  Odom_Data_t odom;
  Imu_Data_t imu;

  ros::Time::setNow(ros::Time(20.0));
  EXPECT_TRUE(odom.feed(validOdom(20.0, 0.0), true, false));
  EXPECT_TRUE(imu.feed(validImu(20.0), true));
  EXPECT_TRUE(controller.calControl(
      odom, imu, desired, output, 0.2, 0.2, dt));

  ros::Time::setNow(ros::Time(20.0 + dt));
  EXPECT_TRUE(odom.feed(validOdom(20.0 + dt, 0.01), true, false));
  EXPECT_TRUE(imu.feed(validImu(20.0 + dt), true));
  EXPECT_TRUE(controller.calControl(
      odom, imu, desired, output, 0.2, 0.2, dt));
  return output;
}
} // namespace

class Se3SafetyTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    ros::Time::init();
  }
};

TEST_F(Se3SafetyTest, SourceTimestampCannotBeRefreshedByCallbackReceipt)
{
  ros::Time::setNow(ros::Time(10.0));
  Imu_Data_t imu;
  EXPECT_TRUE(imu.feed(validImu(8.0), true));
  EXPECT_FALSE(imu.isFreshAt(ros::Time(10.0), 0.2));

  ros::Time::setNow(ros::Time(11.0));
  EXPECT_TRUE(imu.feed(validImu(8.0), true));
  EXPECT_FALSE(imu.isFreshAt(ros::Time(11.0), 0.2));
}

TEST_F(Se3SafetyTest, InvalidQuaternionAndNonFiniteSensorDataAreRejected)
{
  ros::Time::setNow(ros::Time(10.0));
  Odom_Data_t odom;
  nav_msgs::OdometryPtr invalid_odom = validOdom(10.0);
  invalid_odom->pose.pose.orientation.w = 0.0;
  EXPECT_FALSE(odom.feed(invalid_odom, true, false));
  EXPECT_FALSE(odom.recv_new_msg);

  Imu_Data_t imu;
  sensor_msgs::ImuPtr invalid_imu = validImu(10.0);
  invalid_imu->linear_acceleration.x =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(imu.feed(invalid_imu, true));
  EXPECT_FALSE(imu.recv_new_msg);
}

TEST_F(Se3SafetyTest, DerivativeHistoryUsesDtAndResetClearsIt)
{
  const Controller_Output_t slow_step = runDerivativeStep(0.1);
  const Controller_Output_t fast_step = runDerivativeStep(0.01);
  EXPECT_GT(std::abs(fast_step.q.y()), std::abs(slow_step.q.y()) + 1e-3);

  SE3_CONTROLLER controller;
  configureDerivativeController(controller);
  Desired_State_t desired;
  Controller_Output_t output;
  Odom_Data_t odom;
  Imu_Data_t imu;

  ros::Time::setNow(ros::Time(30.0));
  ASSERT_TRUE(odom.feed(validOdom(30.0, 0.0), true, false));
  ASSERT_TRUE(imu.feed(validImu(30.0), true));
  ASSERT_TRUE(controller.calControl(
      odom, imu, desired, output, 0.2, 0.2, 0.1));

  ros::Time::setNow(ros::Time(30.1));
  ASSERT_TRUE(odom.feed(validOdom(30.1, 0.1), true, false));
  ASSERT_TRUE(imu.feed(validImu(30.1), true));
  ASSERT_TRUE(controller.calControl(
      odom, imu, desired, output, 0.2, 0.2, 0.1));
  ASSERT_GT(std::abs(output.q.y()), 1e-3);

  controller.resetIntegral();
  ros::Time::setNow(ros::Time(30.2));
  ASSERT_TRUE(odom.feed(validOdom(30.2, 0.2), true, false));
  ASSERT_TRUE(imu.feed(validImu(30.2), true));
  ASSERT_TRUE(controller.calControl(
      odom, imu, desired, output, 0.2, 0.2, 0.1));
  EXPECT_NEAR(output.q.y(), 0.0, 1e-9);
}

TEST_F(Se3SafetyTest, ControlRejectsStaleImuAndInvalidDesiredState)
{
  SE3_CONTROLLER controller;
  configureDerivativeController(controller);
  Desired_State_t desired;
  Controller_Output_t output;
  Odom_Data_t odom;
  Imu_Data_t imu;

  ros::Time::setNow(ros::Time(40.0));
  ASSERT_TRUE(odom.feed(validOdom(40.0), true, false));
  ASSERT_TRUE(imu.feed(validImu(39.0), true));
  EXPECT_FALSE(controller.calControl(
      odom, imu, desired, output, 0.2, 0.2, 0.01));

  ASSERT_TRUE(imu.feed(validImu(40.0), true));
  desired.p.x() = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(controller.calControl(
      odom, imu, desired, output, 0.2, 0.2, 0.01));
}

TEST_F(Se3SafetyTest, SameEstimatorCanBypassAsynchronousAttitudeAlignment)
{
  SE3_CONTROLLER controller;
  controller.init(0.5, 0.8, 0.0, 0.9, true, false);
  ASSERT_TRUE(controller.setup(
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),
      100.0, 100.0, 100.0,
      1000.0, 1000.0, 1000.0));

  ros::Time::setNow(ros::Time(50.0));
  Odom_Data_t odom;
  Imu_Data_t imu;
  ASSERT_TRUE(odom.feed(validOdom(50.0), true, false));
  sensor_msgs::ImuPtr imu_msg = validImu(50.0);
  const double half_yaw = M_PI / 4.0;
  imu_msg->orientation.z = std::sin(half_yaw);
  imu_msg->orientation.w = std::cos(half_yaw);
  ASSERT_TRUE(imu.feed(imu_msg, true));

  Desired_State_t desired;
  Controller_Output_t aligned_output;
  Controller_Output_t direct_output;
  ASSERT_TRUE(controller.calControl(
      odom, imu, desired, aligned_output, 0.2, 0.2, 0.01, true));
  controller.resetIntegral();
  ASSERT_TRUE(controller.calControl(
      odom, imu, desired, direct_output, 0.2, 0.2, 0.01, false));

  EXPECT_NEAR(
      std::abs(aligned_output.q.z()), std::sin(half_yaw), 1e-9);
  EXPECT_NEAR(std::abs(direct_output.q.z()), 0.0, 1e-9);
  EXPECT_NEAR(std::abs(direct_output.q.w()), 1.0, 1e-9);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
