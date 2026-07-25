#!/usr/bin/env python3

import math
import threading
import unittest

import rospy
import rostest
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Empty
from traj_utils.msg import PolyTraj


class GoalYawTrackingTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._commands = []
        self._heartbeat_pub = rospy.Publisher(
            "/goal_yaw_test/heartbeat", Empty, queue_size=1
        )
        self._trajectory_pub = rospy.Publisher(
            "/goal_yaw_test/planning/trajectory", PolyTraj, queue_size=1, latch=True
        )
        self._command_sub = rospy.Subscriber(
            "/goal_yaw_test/position_cmd",
            PositionCommand,
            self._command_callback,
            queue_size=1000,
        )
        self._heartbeat_timer = rospy.Timer(
            rospy.Duration(0.05), lambda _: self._heartbeat_pub.publish(Empty())
        )

    def tearDown(self):
        self._heartbeat_timer.shutdown()
        self._command_sub.unregister()

    def _command_callback(self, msg):
        with self._lock:
            self._commands.append((rospy.Time.now().to_sec(), msg))

    def _commands_snapshot(self):
        with self._lock:
            return list(self._commands)

    def _wait_for(self, predicate, timeout, message):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if predicate():
                return
            rate.sleep()
        self.fail(message)

    def test_tracks_goal_yaw_and_holds_final_position(self):
        self._wait_for(
            lambda: self._trajectory_pub.get_num_connections() > 0
            and self._heartbeat_pub.get_num_connections() > 0,
            5.0,
            "traj_server did not subscribe to test inputs",
        )

        # traj_server intentionally waits one second before spinning. Keep the
        # synthetic trajectory safely in the future to avoid launch-order flakes.
        start_time = rospy.Time.now() + rospy.Duration(1.5)
        duration = 1.0
        goal_yaw = math.pi / 2.0

        trajectory = PolyTraj()
        trajectory.drone_id = 0
        trajectory.traj_id = 1
        trajectory.start_time = start_time
        trajectory.order = 5
        # Coefficients are stored from t^5 through t^0: x=t, y=0, z=1.
        trajectory.coef_x = [0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        trajectory.coef_y = [0.0] * 6
        trajectory.coef_z = [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        trajectory.duration = [duration]
        trajectory.has_goal_yaw = True
        trajectory.goal_yaw = goal_yaw
        trajectory.goal_position = [duration, 0.0, 1.0]
        self._trajectory_pub.publish(trajectory)

        trajectory_end = start_time.to_sec() + duration
        self._wait_for(
            lambda: rospy.Time.now().to_sec() > trajectory_end + 0.6,
            5.0,
            "test clock did not advance past trajectory end",
        )

        commands = self._commands_snapshot()
        transit_commands = [
            msg for _, msg in commands if 0.1 < msg.position.x < 0.7
        ]
        self.assertTrue(transit_commands, "no in-transit commands were published")
        self.assertLess(
            max(abs(msg.yaw) for msg in transit_commands),
            0.08,
            "yaw should remain path-aligned before the final-yaw switch distance",
        )

        post_end_commands = [
            msg
            for received_at, msg in commands
            if trajectory_end + 0.05 <= received_at <= trajectory_end + 0.55
        ]
        self.assertGreaterEqual(
            len(post_end_commands),
            20,
            "traj_server must continue publishing final hold commands after trajectory end",
        )

        self._wait_for(
            lambda: self._commands_snapshot()
            and abs(self._commands_snapshot()[-1][1].yaw - goal_yaw) < 0.02
            and abs(self._commands_snapshot()[-1][1].yaw_dot) < 0.02,
            5.0,
            "final yaw did not converge",
        )

        commands = self._commands_snapshot()
        max_yaw_rate = max(abs(msg.yaw_dot) for _, msg in commands)
        self.assertLessEqual(max_yaw_rate, math.radians(90.0) + 1e-3)
        max_yaw_rate_step = max(
            abs(commands[index][1].yaw_dot - commands[index - 1][1].yaw_dot)
            for index in range(1, len(commands))
        )
        self.assertLessEqual(
            max_yaw_rate_step,
            math.radians(180.0) * 0.01 + 1e-3,
            "yaw-rate commands must respect the configured acceleration limit",
        )
        final_command = commands[-1][1]
        self.assertAlmostEqual(final_command.position.x, duration, places=3)
        self.assertAlmostEqual(final_command.position.y, 0.0, places=3)
        self.assertAlmostEqual(final_command.position.z, 1.0, places=3)

        # An unconstrained-yaw goal carries has_goal_yaw=False. Even if goal_yaw
        # contains zero, the server must retain path-aligned yaw instead of
        # turning to it near the destination. Continue along +Y so the expected
        # heading is +90 degrees and a mistaken zero-yaw switch is observable.
        with self._lock:
            self._commands = []

        start_time = rospy.Time.now() + rospy.Duration(0.5)
        duration = 1.0
        trajectory = PolyTraj()
        trajectory.drone_id = 0
        trajectory.traj_id = 2
        trajectory.start_time = start_time
        trajectory.order = 5
        trajectory.coef_x = [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        trajectory.coef_y = [0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        trajectory.coef_z = [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        trajectory.duration = [duration]
        trajectory.has_goal_yaw = False
        trajectory.goal_yaw = 0.0
        trajectory.goal_position = [1.0, duration, 1.0]
        self._trajectory_pub.publish(trajectory)

        trajectory_end = start_time.to_sec() + duration
        self._wait_for(
            lambda: rospy.Time.now().to_sec() > trajectory_end + 0.6,
            5.0,
            "test clock did not advance past unconstrained-yaw trajectory end",
        )

        unconstrained_yaw_commands = self._commands_snapshot()
        post_end_commands = [
            msg
            for received_at, msg in unconstrained_yaw_commands
            if trajectory_end + 0.05 <= received_at <= trajectory_end + 0.55
        ]
        self.assertGreaterEqual(
            len(post_end_commands),
            20,
            "traj_server must hold the unconstrained-yaw trajectory after its end",
        )
        for msg in post_end_commands:
            yaw_error = math.atan2(
                math.sin(msg.yaw - math.pi / 2.0),
                math.cos(msg.yaw - math.pi / 2.0),
            )
            self.assertLess(
                abs(yaw_error),
                0.03,
                "unconstrained yaw must retain the path-aligned heading",
            )


if __name__ == "__main__":
    rospy.init_node("test_goal_yaw_tracking")
    rostest.rosrun("diff_planner", "goal_yaw_tracking", GoalYawTrackingTest)
