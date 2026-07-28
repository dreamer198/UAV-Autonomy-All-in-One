#!/usr/bin/env python3

import importlib.util
import math
import pathlib
import threading
from types import SimpleNamespace
import unittest


MODULE_PATH = (
    pathlib.Path(__file__).resolve().parents[1]
    / "scripts"
    / "diff_backend_adapter.py"
)
SPEC = importlib.util.spec_from_file_location("diff_backend_adapter", MODULE_PATH)
ADAPTER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ADAPTER)


class Stamp:
    def __init__(self, secs=1, nsecs=0):
        self.secs = secs
        self.nsecs = nsecs


class Quaternion:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x, self.y, self.z, self.w = x, y, z, w


class Header:
    def __init__(self, frame_id="world", stamp=None):
        self.frame_id = frame_id
        self.stamp = stamp or Stamp()


class Point:
    def __init__(self, x=0.0, y=0.0, z=1.0):
        self.x, self.y, self.z = x, y, z


class Pose:
    def __init__(self):
        self.position = Point()
        self.orientation = Quaternion()


class PoseStamped:
    def __init__(self):
        self.header = Header()
        self.pose = Pose()


class Goal:
    PLAN = 0
    CANCEL = 1

    def __init__(self):
        self.action = self.PLAN
        self.session_id = "session"
        self.goal_id = 1
        self.goal = PoseStamped()
        self.constrain_yaw = True


class DiffAdapterCoreTests(unittest.TestCase):
    def test_accepts_arbitrary_safe_z(self):
        goal = Goal()
        goal.goal.pose.position.z = 1.7
        self.assertEqual(ADAPTER.validate_pose_goal(goal, 0.1, 3.0, 0.33), "")

    def test_rejects_vertical_clearance_boundary(self):
        goal = Goal()
        goal.goal.pose.position.z = 0.43
        self.assertIn(
            "goal z", ADAPTER.validate_pose_goal(goal, 0.1, 3.0, 0.33)
        )

    def test_unconstrained_yaw_accepts_zero_quaternion(self):
        goal = Goal()
        goal.constrain_yaw = False
        goal.goal.pose.orientation = Quaternion(w=0.0)
        self.assertEqual(ADAPTER.validate_pose_goal(goal, 0.1, 3.0, 0.33), "")

    def test_rejects_nan_and_wrong_frame(self):
        goal = Goal()
        goal.goal.pose.position.x = float("nan")
        self.assertIn(
            "NaN", ADAPTER.validate_pose_goal(goal, 0.1, 3.0, 0.33)
        )
        goal.goal.pose.position.x = 0.0
        goal.goal.header.frame_id = "map"
        self.assertIn(
            "world", ADAPTER.validate_pose_goal(goal, 0.1, 3.0, 0.33)
        )

    def test_requires_nonzero_measurement_stamp(self):
        goal = Goal()
        goal.goal.header.stamp = Stamp(0, 0)
        self.assertIn(
            "timestamp", ADAPTER.validate_pose_goal(goal, 0.1, 3.0, 0.33)
        )

    def test_measurement_timestamp_rejects_stale_future_and_replay(self):
        self.assertTrue(
            ADAPTER.measurement_stamp_is_current(
                Stamp(99, 800000000), 100.0, 0.5
            )
        )
        self.assertFalse(
            ADAPTER.measurement_stamp_is_current(
                Stamp(99, 400000000), 100.0, 0.5
            )
        )
        self.assertFalse(
            ADAPTER.measurement_stamp_is_current(
                Stamp(100, 200000000), 100.0, 0.5
            )
        )
        self.assertFalse(
            ADAPTER.measurement_stamp_is_current(
                Stamp(99, 800000000), 100.0, 0.5, 99.8
            )
        )

    def test_recoverable_stop_publishes_stamped_header(self):
        published = []
        adapter = object.__new__(ADAPTER.DiffBackendAdapter)
        adapter.Header = lambda: SimpleNamespace(stamp=0.0, frame_id="")
        adapter.rospy = SimpleNamespace(
            Time=SimpleNamespace(now=lambda: 9.0)
        )
        adapter.lock = threading.RLock()
        adapter.native_goal_stamp = 10.0
        adapter.stop_pub = SimpleNamespace(publish=published.append)

        adapter._publish_recoverable_stop()

        self.assertEqual(len(published), 1)
        self.assertEqual(published[0].stamp, 10.0)
        self.assertEqual(published[0].frame_id, ADAPTER.WORLD_FRAME)

    def test_constrained_yaw_must_settle_before_reached(self):
        adapter = object.__new__(ADAPTER.DiffBackendAdapter)
        adapter.goal = PoseStamped()
        adapter.odom = SimpleNamespace(
            pose=SimpleNamespace(pose=Pose()),
            twist=SimpleNamespace(
                twist=SimpleNamespace(
                    linear=Point(0.0, 0.0, 0.0),
                    angular=Point(0.0, 0.0, 0.0),
                )
            ),
        )
        adapter.native_armable = True
        adapter.constrain_yaw = True
        adapter.goal_tolerance = 0.35
        adapter.reached_velocity_tolerance = 0.2
        adapter.reached_yaw_tolerance = math.radians(5.0)
        adapter.reached_yaw_rate_tolerance = math.radians(10.0)
        adapter.reached_hold_time = 0.0
        adapter.reached_at = None

        adapter.odom.pose.pose.orientation = Quaternion(
            z=math.sin(math.pi / 4.0),
            w=math.cos(math.pi / 4.0),
        )
        self.assertFalse(adapter._update_reached_locked())

        adapter.odom.pose.pose.orientation = Quaternion()
        self.assertTrue(adapter._update_reached_locked())

    def test_hold_motion_is_zeroed_before_publication(self):
        velocity = SimpleNamespace(
            linear=Point(1.0, 2.0, 3.0),
            angular=Point(4.0, 5.0, 6.0),
        )
        acceleration = SimpleNamespace(
            linear=Point(7.0, 8.0, 9.0),
            angular=Point(10.0, 11.0, 12.0),
        )
        point = SimpleNamespace(
            velocities=[velocity],
            accelerations=[acceleration],
        )

        ADAPTER.DiffBackendAdapter._zero_point_motion(point)

        values = (
            velocity.linear.x,
            velocity.linear.y,
            velocity.linear.z,
            velocity.angular.x,
            velocity.angular.y,
            velocity.angular.z,
            acceleration.linear.x,
            acceleration.linear.y,
            acceleration.linear.z,
            acceleration.angular.x,
            acceleration.angular.y,
            acceleration.angular.z,
        )
        self.assertEqual(values, (0.0,) * len(values))


if __name__ == "__main__":
    unittest.main()
