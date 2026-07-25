#!/usr/bin/env python3

import importlib.util
import math
import os
import unittest


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

    def test_parser_defaults_require_live_flight_and_both_goal_consumers(self):
        parser = EXECUTOR._build_parser()
        args = parser.parse_args(["1", "2", "1"])
        EXECUTOR._validate_args(parser, args)
        self.assertFalse(args.allow_disarmed)
        self.assertEqual(args.attitude_setpoint_samples, 10)
        self.assertEqual(args.goal_subscribers, 2)

    def test_localization_fault_latch_is_understood(self):
        reason = EXECUTOR.localization_fault_reason
        self.assertEqual(reason(False), "")
        self.assertEqual(reason("odometry stopped"), "odometry stopped")


if __name__ == "__main__":
    unittest.main()
