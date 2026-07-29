#!/usr/bin/env python3

import importlib.util
import inspect
import os
import sys
import threading
import time
import unittest
from types import SimpleNamespace


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

    def test_disarmed_prearm_mode_matches_arm_executor_behavior(self):
        needs_reset = EXECUTOR.disarmed_mode_requires_reset
        self.assertFalse(needs_reset("AUTO.LOITER", "AUTO.LOITER"))
        self.assertTrue(needs_reset("STABILIZED", "AUTO.LOITER"))
        self.assertTrue(needs_reset("OFFBOARD", "AUTO.LOITER"))
        self.assertTrue(needs_reset("AUTO.LOITER", "STABILIZED"))
        self.assertFalse(needs_reset("STABILIZED", "STABILIZED"))
        self.assertFalse(needs_reset("ALTCTL", "STABILIZED"))

    def test_landing_timeout_allows_px4_to_finish_auto_land(self):
        args = EXECUTOR._build_parser().parse_args(["mission.json"])
        self.assertEqual(args.landing_timeout, 120.0)
        self.assertEqual(args.disarmed_prearm_mode, "STABILIZED")
        self.assertIsNone(args.odom_timeout)
        self.assertEqual(args.state_topic, "/mavros/state")
        self.assertEqual(args.odometry_topic, "/localization/odom")

    def test_waypoint_runner_uses_the_director_state_topic(self):
        source = inspect.getsource(EXECUTOR.SharedMissionExecutor.__init__)
        self.assertIn("state_topic=args.state_topic", source)

    def test_mission_accepts_same_hover_calibration_as_arm(self):
        args = EXECUTOR._build_parser().parse_args(
            ["mission.json", "--px4-hover-thrust", "0.755"]
        )
        self.assertAlmostEqual(args.px4_hover_thrust, 0.755)

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

    def test_flight_director_lands_for_localization_failures(self):
        recovery = EXECUTOR.flight_director_recovery_mode
        self.assertEqual(
            recovery("localization odometry stopped"), "AUTO.LAND"
        )
        self.assertEqual(recovery("planner crashed"), "AUTO.LOITER")

    def test_takeoff_stage_failure_confirms_loiter_without_runner(self):
        calls = []
        executor = EXECUTOR.SharedMissionExecutor.__new__(
            EXECUTOR.SharedMissionExecutor
        )
        executor.condition = threading.Condition()
        executor.state = SimpleNamespace(
            connected=True, armed=True, mode="AUTO.TAKEOFF"
        )
        executor.relative_altitude = 0.4
        executor.state_received_at = time.monotonic()
        executor.config = {"state_timeout": 3.0}
        executor.args = SimpleNamespace(command_timeout=0.2)
        executor.rospy = SimpleNamespace(
            logwarn=lambda *_args: None,
            logerr=lambda *_args: None,
            logerr_throttle=lambda *_args: None,
        )

        def set_mode(**kwargs):
            calls.append(kwargs["custom_mode"])
            executor.state.mode = kwargs["custom_mode"]
            executor.state_received_at = time.monotonic()
            return SimpleNamespace(mode_sent=True)

        executor.set_mode = set_mode
        executor._request_safe_recovery("native takeoff failed")

        self.assertEqual(calls, ["AUTO.LOITER"])

    def test_takeoff_localization_failure_goes_directly_to_land(self):
        calls = []
        executor = EXECUTOR.SharedMissionExecutor.__new__(
            EXECUTOR.SharedMissionExecutor
        )
        executor.condition = threading.Condition()
        executor.state = SimpleNamespace(
            connected=True, armed=True, mode="AUTO.TAKEOFF"
        )
        executor.relative_altitude = 0.4
        executor.state_received_at = time.monotonic()
        executor.config = {"state_timeout": 3.0}
        executor.args = SimpleNamespace(command_timeout=0.2)
        executor.rospy = SimpleNamespace(
            logwarn=lambda *_args: None,
            logerr=lambda *_args: None,
            logerr_throttle=lambda *_args: None,
        )

        def set_mode(**kwargs):
            calls.append(kwargs["custom_mode"])
            executor.state.mode = kwargs["custom_mode"]
            executor.state_received_at = time.monotonic()
            return SimpleNamespace(mode_sent=True)

        executor.set_mode = set_mode
        executor._request_safe_recovery("localization odometry stale")

        self.assertEqual(calls, ["AUTO.LAND"])

    def test_localization_health_fails_closed_and_routes_to_land(self):
        for get_param, odom_age in (
            (lambda *_args: "", 1.0),
            (
                lambda *_args: (_ for _ in ()).throw(
                    RuntimeError("parameter server unavailable")
                ),
                0.0,
            ),
        ):
            with self.subTest(odom_age=odom_age):
                executor = EXECUTOR.SharedMissionExecutor.__new__(
                    EXECUTOR.SharedMissionExecutor
                )
                executor.condition = threading.Condition()
                executor.config = {"odom_timeout": 0.5}
                executor.odom_received_at = time.monotonic() - odom_age
                executor.rospy = SimpleNamespace(get_param=get_param)

                with self.assertRaises(
                    EXECUTOR.FlightDirectorError
                ) as context:
                    executor._check_localization_health()

                reason = str(context.exception)
                self.assertIn("localization", reason)
                self.assertEqual(
                    EXECUTOR.flight_director_recovery_mode(reason),
                    "AUTO.LAND",
                )

    def test_all_flight_wait_loops_check_localization_health(self):
        expected_calls = {
            "_arm_and_start_takeoff": 2,
            "_wait_for_native_takeoff_settle": 1,
            "_settle_before_offboard": 1,
            "_enter_and_verify_offboard": 4,
        }
        for method_name, minimum in expected_calls.items():
            with self.subTest(method=method_name):
                source = inspect.getsource(
                    getattr(EXECUTOR.SharedMissionExecutor, method_name)
                )
                self.assertGreaterEqual(
                    source.count("self._check_localization_health()"),
                    minimum,
                )

    def test_armed_preflight_fails_immediately_for_stale_localization(self):
        executor = EXECUTOR.SharedMissionExecutor.__new__(
            EXECUTOR.SharedMissionExecutor
        )
        executor.condition = threading.Condition()
        executor.abort_requested = False
        executor.rosnode = SimpleNamespace(
            get_node_names=lambda: ["/se3_controller_node"]
        )
        executor.rospy = SimpleNamespace(
            is_shutdown=lambda: False,
            get_param=lambda *_args: "",
        )
        executor.args = SimpleNamespace(
            preflight_timeout=5.0,
            altitude_timeout=0.5,
        )
        executor.config = {"state_timeout": 3.0, "odom_timeout": 0.5}
        now = time.monotonic()
        executor.state = SimpleNamespace(
            connected=True,
            armed=True,
            mode="OFFBOARD",
            system_status=3,
        )
        executor.state_received_at = now
        executor.relative_altitude = 1.0
        executor.altitude_received_at = now
        executor.odom_received_at = now - 1.0
        executor.position_setpoint_count = 0
        executor.position_setpoint_received_at = 0.0

        with self.assertRaisesRegex(
            EXECUTOR.FlightDirectorError, "localization"
        ):
            executor._wait_for_preflight_data()

    def test_armed_preflight_waits_for_first_localization_callback(self):
        executor = EXECUTOR.SharedMissionExecutor.__new__(
            EXECUTOR.SharedMissionExecutor
        )
        executor.condition = threading.Condition()
        executor.abort_requested = False
        executor.rosnode = SimpleNamespace(
            get_node_names=lambda: ["/se3_controller_node"]
        )
        executor.rospy = SimpleNamespace(
            is_shutdown=lambda: False,
            get_param=lambda *_args: "",
        )
        executor.args = SimpleNamespace(
            preflight_timeout=0.5,
            altitude_timeout=0.5,
        )
        executor.config = {"state_timeout": 3.0, "odom_timeout": 0.5}
        now = time.monotonic()
        executor.state = SimpleNamespace(
            connected=True,
            armed=True,
            mode="OFFBOARD",
            system_status=3,
        )
        executor.state_received_at = now
        executor.relative_altitude = 1.0
        executor.altitude_received_at = now
        executor.odom_received_at = 0.0
        executor.position_setpoint_count = 0
        executor.position_setpoint_received_at = 0.0

        def publish_first_odometry():
            time.sleep(0.01)
            with executor.condition:
                executor.odom_received_at = time.monotonic()
                executor.condition.notify_all()

        publisher = threading.Thread(target=publish_first_odometry)
        publisher.start()
        executor._wait_for_preflight_data()
        publisher.join()


if __name__ == "__main__":
    unittest.main()
