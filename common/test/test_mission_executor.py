#!/usr/bin/env python3

import importlib.util
import os
import sys
import unittest


SCRIPT_DIR = os.path.join(os.path.dirname(__file__), "..", "scripts")
sys.path.insert(0, SCRIPT_DIR)
SCRIPT_PATH = os.path.join(SCRIPT_DIR, "mission_executor.py")
SPEC = importlib.util.spec_from_file_location("mission_executor", SCRIPT_PATH)
EXECUTOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(EXECUTOR)


class MissionExecutorTest(unittest.TestCase):
    def test_uses_requested_px4_takeoff_target_without_radius_compensation(self):
        self.assertAlmostEqual(
            EXECUTOR.native_takeoff_target(1.0, 0.8), 1.0
        )

    def test_rejects_invalid_takeoff_target(self):
        with self.assertRaises(EXECUTOR.FlightDirectorError):
            EXECUTOR.native_takeoff_target(1.0, -0.1)

    def test_temporarily_tightens_takeoff_acceptance_radius(self):
        radius = EXECUTOR.temporary_takeoff_acceptance_radius
        self.assertAlmostEqual(radius(0.8, 0.1), 0.1)
        self.assertAlmostEqual(radius(0.05, 0.1), 0.05)
        with self.assertRaises(EXECUTOR.FlightDirectorError):
            radius(0.0, 0.1)

    def test_armed_offboard_preflight_does_not_require_position_stream(self):
        ready = EXECUTOR.preflight_position_stream_ready
        self.assertTrue(ready(True, 0, float("inf"), 0.5))
        self.assertFalse(ready(False, 9, 0.0, 0.5))
        self.assertFalse(ready(False, 10, 0.51, 0.5))
        self.assertTrue(ready(False, 10, 0.1, 0.5))

    def test_native_handoff_matches_arm_requirements(self):
        ready = EXECUTOR.native_takeoff_handoff_ready
        self.assertTrue(ready("AUTO.TAKEOFF", 0.95, 0.1, 1.0, 0.1, 0.2))
        self.assertTrue(ready("AUTO.LOITER", 0.95, -0.1, 1.0, 0.1, 0.2))
        self.assertFalse(ready("OFFBOARD", 0.95, 0.0, 1.0, 0.1, 0.2))
        self.assertFalse(ready("AUTO.TAKEOFF", 1.11, 0.0, 1.0, 0.1, 0.2))
        self.assertFalse(ready("AUTO.LOITER", 0.95, 0.3, 1.0, 0.1, 0.2))

    def test_disarmed_autonomous_modes_are_reset_before_takeoff(self):
        needs_reset = EXECUTOR.disarmed_mode_requires_stabilized
        self.assertTrue(needs_reset("OFFBOARD"))
        self.assertTrue(needs_reset("AUTO.LAND"))
        self.assertTrue(needs_reset("AUTO.LOITER"))
        self.assertFalse(needs_reset("STABILIZED"))
        self.assertFalse(needs_reset("ALTCTL"))

    def test_landing_timeout_allows_px4_to_finish_auto_land(self):
        args = EXECUTOR._build_parser().parse_args(["mission.json"])
        self.assertEqual(args.landing_timeout, 120.0)

    def test_px4_flight_termination_is_rejected_before_arming(self):
        self.assertTrue(EXECUTOR.px4_flight_termination_active(8))
        self.assertFalse(EXECUTOR.px4_flight_termination_active(3))

    def test_mission_uses_shared_localization_fault_contract(self):
        self.assertEqual(
            EXECUTOR.localization_fault_reason(
                {"active": True, "reason": "odometry jumped"}
            ),
            "odometry jumped",
        )


if __name__ == "__main__":
    unittest.main()
