#!/usr/bin/env python3

import importlib.util
import math
import os
import time
import unittest
from types import SimpleNamespace


SCRIPT_PATH = os.path.join(
    os.path.dirname(__file__), "..", "scripts", "goal_executor.py"
)
SPEC = importlib.util.spec_from_file_location("goal_executor", SCRIPT_PATH)
EXECUTOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(EXECUTOR)


class GoalExecutorTest(unittest.TestCase):
    def test_unconstrained_yaw_uses_zero_norm_planner_contract(self):
        self.assertEqual(EXECUTOR.goal_orientation(None), (0.0, 0.0))

    def test_converts_goal_yaw_to_quaternion(self):
        qz, qw = EXECUTOR.goal_orientation(90.0)
        self.assertAlmostEqual(qz, math.sqrt(0.5))
        self.assertAlmostEqual(qw, math.sqrt(0.5))

    def test_rejects_nonfinite_goal_values(self):
        with self.assertRaises(EXECUTOR.GoalExecutorError):
            EXECUTOR.validate_goal_coordinates(float("nan"), 0.0, 1.0)
        with self.assertRaises(EXECUTOR.GoalExecutorError):
            EXECUTOR.goal_orientation(float("inf"))

    def test_vertical_bounds_include_obstacle_inflation(self):
        minimum_z, maximum_z = EXECUTOR.vertical_clearance_bounds(
            0.1, 3.0, 0.33
        )
        self.assertAlmostEqual(minimum_z, 0.43)
        self.assertAlmostEqual(maximum_z, 2.67)
        with self.assertRaises(EXECUTOR.GoalExecutorError):
            EXECUTOR.vertical_clearance_bounds(0.1, 0.5, 0.3)

    def test_parser_defaults_require_live_flight_and_gateway_goal_consumer(self):
        parser = EXECUTOR._build_parser()
        args = parser.parse_args(["1", "2", "1"])
        EXECUTOR._validate_args(parser, args)
        self.assertFalse(args.allow_disarmed)
        self.assertEqual(args.attitude_setpoint_samples, 10)
        self.assertEqual(args.position_setpoint_samples, 10)
        self.assertEqual(
            args.position_setpoint_topic, "/mavros/setpoint_position/local"
        )
        self.assertEqual(args.goal_subscribers, 1)
        self.assertEqual(args.state_timeout, 3.0)
        self.assertEqual(args.planner_status_topic, "/planning/status")

    def test_localization_fault_latch_is_understood(self):
        reason = EXECUTOR.localization_fault_reason
        self.assertEqual(reason(False), "")
        self.assertEqual(reason("odometry stopped"), "odometry stopped")

    def test_stale_mavros_state_never_authorizes_a_goal(self):
        executor = EXECUTOR.SharedGoalExecutor.__new__(
            EXECUTOR.SharedGoalExecutor
        )
        executor.rospy = SimpleNamespace(get_param=lambda *_args: "")
        executor.state = SimpleNamespace(
            connected=True, armed=True, mode="OFFBOARD"
        )
        executor.state_received_at = time.monotonic() - 4.0
        executor.args = SimpleNamespace(state_timeout=3.0)

        self.assertEqual(
            executor._readiness_reason(time.monotonic()),
            "waiting for fresh MAVROS state",
        )

    def test_position_hold_authorizes_goal_before_se3_attitude_handoff(self):
        executor = EXECUTOR.SharedGoalExecutor.__new__(
            EXECUTOR.SharedGoalExecutor
        )
        executor.args = SimpleNamespace(
            position_setpoint_samples=10,
            attitude_setpoint_samples=10,
            stream_gap_timeout=0.5,
        )
        now = time.monotonic()
        executor.position_setpoint_count = 10
        executor.position_setpoint_received_at = now
        executor.attitude_setpoint_count = 0
        executor.attitude_setpoint_received_at = 0.0
        self.assertTrue(executor._control_stream_ready(now))

        executor.position_setpoint_received_at = now - 1.0
        self.assertFalse(executor._control_stream_ready(now))

        executor.attitude_setpoint_count = 10
        executor.attitude_setpoint_received_at = now
        self.assertTrue(executor._control_stream_ready(now))


if __name__ == "__main__":
    unittest.main()
