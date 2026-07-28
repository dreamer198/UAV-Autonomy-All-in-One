#include <gtest/gtest.h>
#include <plan_manage/collision_range.h>

using fast_planner::classifyCollisionRange;
using fast_planner::CollisionRangeState;

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

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
