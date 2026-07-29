#include <gtest/gtest.h>
#include <limits>
#include <plan_manage/collision_range.h>
#include <path_searching/topology_safety.h>

using fast_planner::classifyCollisionRange;
using fast_planner::InitialClearanceEscape;
using fast_planner::CollisionRangeState;
using fast_planner::isObservedClearance;
using fast_planner::sampleTimesIncludingEnd;
using fast_planner::topologyEdgeClearance;
using fast_planner::violatesRequiredClearance;

TEST(CollisionRange, TreatsNoTransitionsAsClear) {
  EXPECT_EQ(CollisionRangeState::CLEAR, classifyCollisionRange(0U, 0U));
}

TEST(CollisionRange, AcceptsOneOrMoreBoundedIntervals) {
  EXPECT_EQ(CollisionRangeState::BOUNDED, classifyCollisionRange(1U, 1U));
  EXPECT_EQ(CollisionRangeState::BOUNDED, classifyCollisionRange(3U, 3U));
}

TEST(CollisionRange, DetectsAReferenceThatEndsInsideAnObstacle) {
  EXPECT_EQ(
      CollisionRangeState::ENDS_IN_OBSTACLE,
      classifyCollisionRange(1U, 0U));
  EXPECT_EQ(
      CollisionRangeState::ENDS_IN_OBSTACLE,
      classifyCollisionRange(3U, 2U));
}

TEST(CollisionRange, RejectsImpossibleTransitionCounts) {
  EXPECT_EQ(CollisionRangeState::INVALID, classifyCollisionRange(0U, 1U));
  EXPECT_EQ(CollisionRangeState::INVALID, classifyCollisionRange(1U, 2U));
  EXPECT_EQ(CollisionRangeState::INVALID, classifyCollisionRange(3U, 1U));
}

TEST(CollisionRange, SamplesTheExactEndOfANonIntegralTimeSpan) {
  const std::vector<double> samples =
      sampleTimesIncludingEnd(0.0, 1.03, 0.05);

  ASSERT_GT(samples.size(), 2U);
  EXPECT_DOUBLE_EQ(0.0, samples.front());
  EXPECT_DOUBLE_EQ(1.03, samples.back());
  for (std::size_t index = 1; index < samples.size(); ++index) {
    EXPECT_GT(samples[index], samples[index - 1]);
    EXPECT_LE(samples[index] - samples[index - 1], 0.05 + 1.0e-12);
  }
}

TEST(CollisionRange, SamplesAZeroDurationSpanOnce) {
  const std::vector<double> samples =
      sampleTimesIncludingEnd(2.0, 2.0, 0.05);

  ASSERT_EQ(1U, samples.size());
  EXPECT_DOUBLE_EQ(2.0, samples.front());
}

TEST(CollisionRange, UsesTheConfiguredTopologyClearanceForGoalRecovery) {
  EXPECT_TRUE(violatesRequiredClearance(0.49, 0.50));
  EXPECT_TRUE(violatesRequiredClearance(0.50, 0.50));
  EXPECT_FALSE(violatesRequiredClearance(0.51, 0.50));
}

TEST(CollisionRange, TreatsInvalidClearanceMeasurementsAsUnsafe) {
  EXPECT_TRUE(violatesRequiredClearance(
      std::numeric_limits<double>::quiet_NaN(), 0.50));
  EXPECT_TRUE(violatesRequiredClearance(1.0, 0.0));
}

TEST(CollisionRange, RejectsTheUninitializedEsdfDistanceAsAnObservation) {
  EXPECT_TRUE(isObservedClearance(0.75));
  EXPECT_FALSE(isObservedClearance(10000.0));
  EXPECT_FALSE(isObservedClearance(
      std::numeric_limits<double>::infinity()));
}

TEST(CollisionRange, KeepsPrmEdgesAtNearlyTheFullConfiguredClearance) {
  const double edge_clearance = topologyEdgeClearance(0.30, 0.20);

  EXPECT_GT(edge_clearance, 0.299);
  EXPECT_LT(edge_clearance, 0.30);
}

TEST(CollisionRange, NeverUsesLessThanOneVoxelForPrmEdges) {
  EXPECT_DOUBLE_EQ(0.20, topologyEdgeClearance(0.10, 0.20));
  EXPECT_DOUBLE_EQ(0.0, topologyEdgeClearance(0.0, 0.20));
}

TEST(CollisionRange, KeepsNormalTrajectoriesAtTheConfiguredClearance) {
  InitialClearanceEscape validator(0.40, 0.30, 0.03, 0.60);

  EXPECT_TRUE(validator.accept(0.40, 0.0));
  EXPECT_TRUE(validator.complete());
  EXPECT_TRUE(validator.accept(0.30, 0.20));
  EXPECT_FALSE(validator.accept(0.29, 0.30));
}

TEST(CollisionRange, AllowsOnlyAShortInitialClearanceEscape) {
  InitialClearanceEscape validator(0.28, 0.30, 0.03, 0.60);

  EXPECT_TRUE(validator.accept(0.28, 0.0));
  EXPECT_TRUE(validator.escaping());
  EXPECT_TRUE(validator.accept(0.27, 0.10));
  EXPECT_TRUE(validator.accept(0.29, 0.30));
  EXPECT_TRUE(validator.accept(0.31, 0.50));
  EXPECT_TRUE(validator.complete());
}

TEST(CollisionRange, RejectsAnEscapeThatMovesMateriallyCloser) {
  InitialClearanceEscape validator(0.28, 0.30, 0.03, 0.60);

  EXPECT_TRUE(validator.accept(0.28, 0.0));
  EXPECT_FALSE(validator.accept(0.24, 0.10));
  EXPECT_FALSE(validator.complete());
}

TEST(CollisionRange, RejectsAnEscapeThatTakesTooLong) {
  InitialClearanceEscape validator(0.28, 0.30, 0.03, 0.60);

  EXPECT_TRUE(validator.accept(0.28, 0.0));
  EXPECT_FALSE(validator.accept(0.29, 0.61));
  EXPECT_FALSE(validator.complete());
}

TEST(CollisionRange, RequiresTheEscapeToRegainFullClearance) {
  InitialClearanceEscape validator(0.28, 0.30, 0.03, 0.60);

  EXPECT_TRUE(validator.accept(0.28, 0.0));
  EXPECT_TRUE(validator.accept(0.29, 0.50));
  EXPECT_FALSE(validator.complete());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
