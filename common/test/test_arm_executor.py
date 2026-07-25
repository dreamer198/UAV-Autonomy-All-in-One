#!/usr/bin/env python3

import importlib.util
import os
import unittest


SCRIPT_PATH = os.path.join(
    os.path.dirname(__file__), "..", "scripts", "arm_executor.py"
)
SPEC = importlib.util.spec_from_file_location("arm_executor", SCRIPT_PATH)
EXECUTOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(EXECUTOR)


class ArmExecutorTest(unittest.TestCase):
    def test_uses_requested_px4_takeoff_target_without_radius_compensation(self):
        self.assertAlmostEqual(
            EXECUTOR.native_takeoff_target(1.0, 0.8), 1.0
        )

    def test_rejects_invalid_takeoff_target(self):
        for height, radius in ((0.0, 0.8), (1.0, -0.1), (float("nan"), 0.8)):
            with self.subTest(height=height, radius=radius):
                with self.assertRaises(EXECUTOR.ArmExecutorError):
                    EXECUTOR.native_takeoff_target(height, radius)

    def test_temporarily_tightens_takeoff_acceptance_radius(self):
        radius = EXECUTOR.temporary_takeoff_acceptance_radius
        self.assertAlmostEqual(radius(0.8, 0.1), 0.1)
        self.assertAlmostEqual(radius(0.05, 0.1), 0.05)
        for current, tolerance in ((0.0, 0.1), (0.8, 0.0)):
            with self.subTest(current=current, tolerance=tolerance):
                with self.assertRaises(EXECUTOR.ArmExecutorError):
                    radius(current, tolerance)

    def test_parser_defaults_preserve_required_safety_samples(self):
        parser = EXECUTOR._build_parser()
        args = parser.parse_args([])
        EXECUTOR._validate_args(parser, args)
        self.assertEqual(args.position_setpoint_samples, 10)
        self.assertEqual(args.attitude_setpoint_samples, 5)
        self.assertEqual(args.takeoff_stable_time, 0.5)
        self.assertEqual(args.takeoff_max_vertical_speed, 0.2)
        self.assertEqual(args.odometry_topic, "/localization/odom")
        self.assertEqual(args.takeoff_altitude_field, "relative")
        self.assertEqual(args.disarmed_prearm_mode, "STABILIZED")
        self.assertIsNone(args.px4_hover_thrust)

    def test_native_handoff_accepts_settled_takeoff_or_loiter(self):
        ready = EXECUTOR.native_takeoff_handoff_ready
        self.assertTrue(ready("AUTO.TAKEOFF", 0.95, 0.1, 1.0, 0.1, 0.2))
        self.assertTrue(ready("AUTO.LOITER", 0.95, 0.1, 1.0, 0.1, 0.2))
        self.assertFalse(ready("OFFBOARD", 0.95, 0.1, 1.0, 0.1, 0.2))
        self.assertFalse(ready("AUTO.LOITER", 0.89, 0.1, 1.0, 0.1, 0.2))
        self.assertFalse(ready("AUTO.TAKEOFF", 1.11, 0.1, 1.0, 0.1, 0.2))
        self.assertFalse(ready("AUTO.LOITER", 0.95, 0.21, 1.0, 0.1, 0.2))
        self.assertFalse(
            ready("AUTO.LOITER", 0.95, float("nan"), 1.0, 0.1, 0.2)
        )

    def test_simulation_auto_altitude_uses_value_closest_to_target(self):
        select = EXECUTOR.select_takeoff_altitude
        self.assertEqual(select("auto", 0.78, 0.96, 1.0), ("relative", 0.96))
        self.assertEqual(select("auto", 0.92, 0.75, 1.0), ("local", 0.92))
        self.assertEqual(select("relative", 0.92, 0.75, 1.0), ("relative", 0.75))
        self.assertEqual(
            select("auto", float("nan"), None, 1.0), ("auto", None)
        )

    def test_disarmed_autonomous_modes_are_reset_before_takeoff(self):
        needs_reset = EXECUTOR.disarmed_mode_requires_stabilized
        self.assertTrue(needs_reset("OFFBOARD"))
        self.assertTrue(needs_reset("AUTO.LAND"))
        self.assertTrue(needs_reset("AUTO.LOITER"))
        self.assertFalse(needs_reset("STABILIZED"))
        self.assertFalse(needs_reset("POSCTL"))

    def test_simulation_prearm_mode_does_not_require_manual_input(self):
        needs_reset = EXECUTOR.disarmed_mode_requires_reset
        self.assertFalse(needs_reset("AUTO.LOITER", "AUTO.LOITER"))
        self.assertTrue(needs_reset("STABILIZED", "AUTO.LOITER"))
        self.assertTrue(needs_reset("OFFBOARD", "AUTO.LOITER"))
        self.assertTrue(needs_reset("AUTO.LOITER", "STABILIZED"))
        self.assertFalse(needs_reset("POSCTL", "STABILIZED"))

    def test_px4_flight_termination_is_rejected_before_arming(self):
        self.assertTrue(EXECUTOR.px4_flight_termination_active(8))
        self.assertFalse(EXECUTOR.px4_flight_termination_active(3))

    def test_localization_fault_latch_is_understood(self):
        reason = EXECUTOR.localization_fault_reason
        self.assertEqual(reason(""), "")
        self.assertEqual(
            reason({"active": True, "reason": "lidar link lost"}),
            "lidar link lost",
        )
        self.assertEqual(reason({"active": False, "reason": "old"}), "")


if __name__ == "__main__":
    unittest.main()
