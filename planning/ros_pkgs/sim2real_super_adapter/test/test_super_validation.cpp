#include <sim2real_super_adapter/validation.h>

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <cmath>
#include <limits>
#include <string>

namespace {

geometry_msgs::PoseStamped makeGoal(double x, double y, double z) {
  geometry_msgs::PoseStamped goal;
  goal.header.frame_id = "world";
  goal.header.stamp = ros::Time(10.0);
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = z;
  return goal;
}

TEST(SuperValidation, PreservesZeroQuaternionYawContract) {
  geometry_msgs::PoseStamped goal = makeGoal(1.0, 2.0, 1.0);
  std::string reason;
  EXPECT_TRUE(sim2real_super_adapter::finiteGoalPose(goal, false, &reason));
  EXPECT_FALSE(sim2real_super_adapter::finiteGoalPose(goal, true, &reason));

  goal.pose.orientation.z = std::sin(0.25);
  goal.pose.orientation.w = std::cos(0.25);
  EXPECT_TRUE(sim2real_super_adapter::finiteGoalPose(goal, true, &reason));
  EXPECT_FALSE(sim2real_super_adapter::finiteGoalPose(goal, false, &reason));
}

TEST(SuperValidation, CorrelatesNativeGoalWithOriginalPoseStamp) {
  sim2real_planning_msgs::PlannerGoal goal;
  goal.header.stamp = ros::Time(20.0);
  goal.goal = makeGoal(1.0, 2.0, 1.0);
  EXPECT_EQ(ros::Time(10.0),
            sim2real_super_adapter::nativeGoalCorrelationStamp(goal));
}

TEST(SuperValidation, RejectsStaleFutureAndReplayedMeasurements) {
  const ros::Time now(10.0);
  EXPECT_TRUE(sim2real_super_adapter::measurementStampIsCurrent(
      ros::Time(9.8), now, 0.5, ros::Time(9.7)));
  EXPECT_FALSE(sim2real_super_adapter::measurementStampIsCurrent(
      ros::Time(9.4), now, 0.5, ros::Time()));
  EXPECT_FALSE(sim2real_super_adapter::measurementStampIsCurrent(
      ros::Time(10.6), now, 0.5, ros::Time()));
  EXPECT_FALSE(sim2real_super_adapter::measurementStampIsCurrent(
      ros::Time(9.8), now, 0.5, ros::Time(9.8)));
  EXPECT_FALSE(sim2real_super_adapter::measurementStampIsCurrent(
      ros::Time(9.7), now, 0.5, ros::Time(9.8)));
}

TEST(SuperValidation, ClassifiesNativeTrajectoryIds) {
  using Decision = sim2real_super_adapter::NativeTrajectoryIdDecision;
  EXPECT_EQ(Decision::ACCEPT_NEW,
            sim2real_super_adapter::classifyNativeTrajectoryId(8U, 7U, 7U));
  EXPECT_EQ(Decision::IGNORE_DUPLICATE,
            sim2real_super_adapter::classifyNativeTrajectoryId(7U, 7U, 7U));
  EXPECT_EQ(Decision::FAULT_BACKWARDS_OR_REPLAY,
            sim2real_super_adapter::classifyNativeTrajectoryId(6U, 7U, 7U));
  EXPECT_EQ(Decision::FAULT_BACKWARDS_OR_REPLAY,
            sim2real_super_adapter::classifyNativeTrajectoryId(7U, 7U, 0U));
}

TEST(SuperValidation, ReplanHoldCanExitOnlyForHigherTrajectoryId) {
  using Decision = sim2real_super_adapter::NativeTrajectoryIdDecision;
  EXPECT_EQ(Decision::ACCEPT_NEW,
            sim2real_super_adapter::classifyNativeTrajectoryId(43U, 42U, 42U));
  EXPECT_EQ(Decision::IGNORE_DUPLICATE,
            sim2real_super_adapter::classifyNativeTrajectoryId(42U, 42U, 42U));
  EXPECT_EQ(Decision::FAULT_BACKWARDS_OR_REPLAY,
            sim2real_super_adapter::classifyNativeTrajectoryId(41U, 42U, 42U));
}

TEST(SuperValidation, BackupRequiresAnEarlierNormalCommandInTheSameGoal) {
  using Decision = sim2real_super_adapter::NativeCommandDecision;
  EXPECT_EQ(Decision::NORMAL,
            sim2real_super_adapter::classifyNativeCommand(1U, false));
  EXPECT_EQ(Decision::REJECT_INVALID_OR_UNAUTHORIZED,
            sim2real_super_adapter::classifyNativeCommand(2U, false));
  EXPECT_EQ(Decision::BRAKE,
            sim2real_super_adapter::classifyNativeCommand(2U, true));
  EXPECT_EQ(Decision::REJECT_INVALID_OR_UNAUTHORIZED,
            sim2real_super_adapter::classifyNativeCommand(3U, true));
}

TEST(SuperValidation, OnlineGoalHandoffRequiresAnActiveNativeMotionTrajectory) {
  EXPECT_TRUE(sim2real_super_adapter::nativeOnlineGoalHandoffAllowed(
      true, true, true, 42U, true, true, false, false, false, 4U));
  EXPECT_FALSE(sim2real_super_adapter::nativeOnlineGoalHandoffAllowed(
      false, true, true, 42U, true, true, false, false, false, 4U));
  EXPECT_FALSE(sim2real_super_adapter::nativeOnlineGoalHandoffAllowed(
      true, true, true, 0U, true, true, false, false, false, 4U));
  EXPECT_FALSE(sim2real_super_adapter::nativeOnlineGoalHandoffAllowed(
      true, true, true, 42U, false, true, false, false, false, 4U));
  EXPECT_FALSE(sim2real_super_adapter::nativeOnlineGoalHandoffAllowed(
      true, true, true, 42U, true, true, true, false, false, 4U));
  EXPECT_FALSE(sim2real_super_adapter::nativeOnlineGoalHandoffAllowed(
      true, true, true, 42U, true, true, false, true, false, 4U));
  EXPECT_FALSE(sim2real_super_adapter::nativeOnlineGoalHandoffAllowed(
      true, true, true, 42U, true, true, false, false, false, 3U));
}

TEST(SuperValidation, FollowToWaitRequiresAnAuthorizedCommittedEndpoint) {
  EXPECT_TRUE(sim2real_super_adapter::nativeProgressIndicatesFinished(
      4U, 1U, true, true, true));
  EXPECT_FALSE(sim2real_super_adapter::nativeProgressIndicatesFinished(
      3U, 1U, true, true, true));
  EXPECT_FALSE(sim2real_super_adapter::nativeProgressIndicatesFinished(
      4U, 1U, true, false, true));
  EXPECT_FALSE(sim2real_super_adapter::nativeProgressIndicatesFinished(
      4U, 1U, true, true, false));
}

TEST(SuperValidation, FollowToGenerateStartsOnlySafeBoundedReplanHold) {
  EXPECT_TRUE(sim2real_super_adapter::nativeProgressStartsReplanHold(
      4U, 3U, true, true, true));
  EXPECT_FALSE(sim2real_super_adapter::nativeProgressStartsReplanHold(
      4U, 3U, false, true, true));
  EXPECT_FALSE(sim2real_super_adapter::nativeProgressStartsReplanHold(
      4U, 3U, true, false, true));
  EXPECT_FALSE(sim2real_super_adapter::nativeProgressStartsReplanHold(
      4U, 3U, true, true, false));
  EXPECT_FALSE(sim2real_super_adapter::nativeProgressStartsReplanHold(
      3U, 4U, true, true, true));
}

TEST(SuperValidation, ReplanHoldSuppressesOnlyGenerateStateStreamWatchdog) {
  EXPECT_TRUE(
      sim2real_super_adapter::nativeReplanHoldPermitsMissingCommand(true, 3U));
  EXPECT_FALSE(
      sim2real_super_adapter::nativeReplanHoldPermitsMissingCommand(true, 4U));
  EXPECT_FALSE(
      sim2real_super_adapter::nativeReplanHoldPermitsMissingCommand(true, 1U));
  EXPECT_FALSE(
      sim2real_super_adapter::nativeReplanHoldPermitsMissingCommand(false, 3U));
}

TEST(SuperValidation, GenerateToWaitSafelyFinishesAnExistingReplanHold) {
  EXPECT_TRUE(
      sim2real_super_adapter::nativeProgressFinishesReplanHold(3U, 1U, true));
  EXPECT_FALSE(
      sim2real_super_adapter::nativeProgressFinishesReplanHold(3U, 1U, false));
  EXPECT_FALSE(
      sim2real_super_adapter::nativeProgressFinishesReplanHold(4U, 1U, true));
  EXPECT_FALSE(
      sim2real_super_adapter::nativeProgressFinishesReplanHold(3U, 4U, true));
}

TEST(SuperValidation, NativeWaitCanFinishAnAlreadyCloseNewGoal) {
  EXPECT_TRUE(
      sim2real_super_adapter::nativeProgressFinishesCloseGoalWithoutTrajectory(
          3U, 1U, true, false));
  EXPECT_FALSE(
      sim2real_super_adapter::nativeProgressFinishesCloseGoalWithoutTrajectory(
          3U, 1U, false, false));
  EXPECT_FALSE(
      sim2real_super_adapter::nativeProgressFinishesCloseGoalWithoutTrajectory(
          3U, 1U, true, true));
  EXPECT_TRUE(
      sim2real_super_adapter::nativeProgressFinishesCloseGoalWithoutTrajectory(
          4U, 1U, true, false));
}

TEST(SuperValidation, PublicTrajectoryIdsRemainMonotonicAfterSyntheticHold) {
  EXPECT_EQ(42U, sim2real_super_adapter::nextPublicTrajectoryId(40U, 42U));
  EXPECT_EQ(43U, sim2real_super_adapter::nextPublicTrajectoryId(42U, 42U));
  EXPECT_EQ(0U, sim2real_super_adapter::nextPublicTrajectoryId(
                    std::numeric_limits<uint64_t>::max(), 1U));
}

TEST(SuperValidation, ReplanHoldHasStrictPlanningTimeoutBound) {
  EXPECT_FALSE(
      sim2real_super_adapter::nativeReplanHoldTimedOut(true, 9.99, 10.0));
  EXPECT_FALSE(
      sim2real_super_adapter::nativeReplanHoldTimedOut(true, 10.0, 10.0));
  EXPECT_TRUE(
      sim2real_super_adapter::nativeReplanHoldTimedOut(true, 10.01, 10.0));
  EXPECT_FALSE(
      sim2real_super_adapter::nativeReplanHoldTimedOut(false, 10.01, 10.0));
  EXPECT_FALSE(sim2real_super_adapter::nativeReplanHoldTimedOut(
      true, std::numeric_limits<double>::quiet_NaN(), 10.0));
}

TEST(SuperValidation, ConvertsBodyVelocityToWorld) {
  const Eigen::Quaterniond quarter_turn(
      Eigen::AngleAxisd(0.5 * std::acos(-1.0), Eigen::Vector3d::UnitZ()));
  const Eigen::Vector3d world = sim2real_super_adapter::bodyVectorToWorld(
      quarter_turn, Eigen::Vector3d::UnitX());
  EXPECT_NEAR(world.x(), 0.0, 1.0e-12);
  EXPECT_NEAR(world.y(), 1.0, 1.0e-12);
  EXPECT_NEAR(world.z(), 0.0, 1.0e-12);
}

TEST(SuperValidation, SelfFilterUsesBodyAlignedCylinder) {
  const Eigen::Quaterniond body_in_world(
      Eigen::AngleAxisd(0.5 * std::acos(-1.0), Eigen::Vector3d::UnitY()));
  const Eigen::Vector3d body_position(1.0, 2.0, 3.0);
  const Eigen::Vector3d inside =
      body_position + body_in_world * Eigen::Vector3d(0.1, 0.1, 0.0);
  const Eigen::Vector3d outside =
      body_position + body_in_world * Eigen::Vector3d(0.5, 0.0, 0.0);
  EXPECT_TRUE(sim2real_super_adapter::pointInsideBodyExclusionCylinder(
      inside, body_position, body_in_world, 0.35, -0.2, 0.2));
  EXPECT_FALSE(sim2real_super_adapter::pointInsideBodyExclusionCylinder(
      outside, body_position, body_in_world, 0.35, -0.2, 0.2));
}

TEST(SuperValidation, SettleGateUsesLinearAndFullAngularNorms) {
  EXPECT_TRUE(sim2real_super_adapter::measuredStateIsSettled(
      Eigen::Vector3d(0.1, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.1), 0.2,
      0.2));
  EXPECT_FALSE(sim2real_super_adapter::measuredStateIsSettled(
      Eigen::Vector3d(0.21, 0.0, 0.0), Eigen::Vector3d::Zero(), 0.2, 0.2));
  EXPECT_FALSE(sim2real_super_adapter::measuredStateIsSettled(
      Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.21, 0.0), 0.2, 0.2));
}

TEST(SuperValidation, ReachedUsesEffectiveGoalAndMeasuredYaw) {
  geometry_msgs::PoseStamped goal = makeGoal(1.0, 2.0, 1.0);
  goal.pose.orientation.z = std::sin(0.25);
  goal.pose.orientation.w = std::cos(0.25);
  const Eigen::Quaterniond body_in_world(
      Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()));
  EXPECT_TRUE(sim2real_super_adapter::measuredStateSatisfiesGoal(
      Eigen::Vector3d(1.1, 2.0, 1.0), Eigen::Vector3d(0.1, 0.0, 0.0),
      body_in_world, Eigen::Vector3d(0.0, 0.0, 0.05), goal, true, 0.35, 0.2,
      0.1, 0.2));
  EXPECT_FALSE(sim2real_super_adapter::measuredStateSatisfiesGoal(
      Eigen::Vector3d(1.5, 2.0, 1.0), Eigen::Vector3d::Zero(), body_in_world,
      Eigen::Vector3d::Zero(), goal, true, 0.35, 0.2, 0.1, 0.2));
}

} // namespace

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
