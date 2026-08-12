#!/usr/bin/env python3

import importlib.util
import inspect
import os
import tempfile
import time
import unittest
from types import SimpleNamespace


SCRIPT = os.path.join(
    os.path.dirname(__file__), "..", "scripts", "flight_command_server.py"
)
SPEC = importlib.util.spec_from_file_location("flight_command_server", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
PACKAGE_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..")
)
REPOSITORY = os.path.abspath(
    os.environ.get(
        "SIM2REAL_PROJECT_ROOT",
        os.path.join(PACKAGE_ROOT, ".."),
    )
)


def state_values(**updates):
    values = {
        "connected": True,
        "armed": False,
        "mode": "STABILIZED",
        "state_age": 0.1,
        "landed_state": MODULE.ON_GROUND,
        "extended_state_age": 0.1,
        "state_timeout": 3.0,
    }
    values.update(updates)
    return values


class DummyCondition:
    def __enter__(self):
        return self

    def __exit__(self, _type, _value, _traceback):
        return False

    @staticmethod
    def wait(_timeout):
        return None


class FakeRosException(Exception):
    pass


class FakeServiceException(Exception):
    pass


class FlightCommandPureSafetyTest(unittest.TestCase):
    def test_takeoff_height_has_an_authoritative_server_range(self):
        validate = MODULE.validate_command_request
        for height in (0.5, 1.5, 2.5):
            self.assertEqual(
                validate(MODULE.COMMAND_TAKEOFF, height),
                (MODULE.COMMAND_TAKEOFF, height),
            )
        for height in (-1.0, 0.49, 2.51, float("nan"), float("inf")):
            with self.assertRaises(MODULE.FlightRequestError) as context:
                validate(MODULE.COMMAND_TAKEOFF, height)
            self.assertEqual(context.exception.reason, "invalid_request")

    def test_land_ignores_a_finite_unused_height_but_rejects_bad_payloads(self):
        self.assertEqual(
            MODULE.validate_command_request(MODULE.COMMAND_LAND, 0.0),
            (MODULE.COMMAND_LAND, None),
        )
        for command, height in (
            (0, 0.0),
            (3, 0.0),
            ("bad", 0.0),
            (2, float("nan")),
            (2, True),
        ):
            with self.assertRaises(MODULE.FlightRequestError) as context:
                MODULE.validate_command_request(command, height)
            self.assertEqual(context.exception.reason, "invalid_request")

    def test_takeoff_requires_fresh_connected_disarmed_ground_state(self):
        classify = MODULE.vehicle_request_kind
        base = state_values(command=MODULE.COMMAND_TAKEOFF)
        self.assertEqual(classify(**base), "disarmed_ground")
        for update, reason in (
            ({"connected": False}, "link_unavailable"),
            ({"state_age": 3.1}, "state_stale"),
            ({"state_age": -0.1}, "state_stale"),
            ({"extended_state_age": 3.1}, "state_stale"),
            ({"extended_state_age": -0.1}, "state_stale"),
            ({"armed": True}, "state_rejected"),
            ({"landed_state": MODULE.IN_AIR}, "state_rejected"),
        ):
            values = dict(base)
            values.update(update)
            with self.assertRaises(MODULE.FlightRequestError) as context:
                classify(**values)
            self.assertEqual(context.exception.reason, reason)

    def test_land_requires_fresh_armed_airborne_autonomous_state(self):
        classify = MODULE.vehicle_request_kind
        for mode, landed_state in (
            ("OFFBOARD", MODULE.IN_AIR),
            ("AUTO.TAKEOFF", MODULE.TAKING_OFF),
            ("AUTO.LOITER", MODULE.IN_AIR),
            ("AUTO.LAND", MODULE.LANDING),
        ):
            values = state_values(
                command=MODULE.COMMAND_LAND,
                armed=True,
                mode=mode,
                landed_state=landed_state,
            )
            self.assertEqual(classify(**values), "armed_airborne")

        base = state_values(
            command=MODULE.COMMAND_LAND,
            armed=True,
            mode="OFFBOARD",
            landed_state=MODULE.IN_AIR,
        )
        for update, reason in (
            ({"connected": False}, "link_unavailable"),
            ({"state_age": 3.1}, "state_stale"),
            ({"extended_state_age": 3.1}, "state_stale"),
            ({"armed": False}, "state_rejected"),
            ({"landed_state": MODULE.ON_GROUND}, "state_rejected"),
            ({"landed_state": 0}, "state_rejected"),
            ({"mode": "POSCTL"}, "state_rejected"),
            ({"mode": "STABILIZED"}, "state_rejected"),
        ):
            values = dict(base)
            values.update(update)
            with self.assertRaises(MODULE.FlightRequestError) as context:
                classify(**values)
            self.assertEqual(context.exception.reason, reason)

    def test_takeoff_success_requires_fresh_armed_offboard(self):
        ready = MODULE.takeoff_handoff_ready
        base = dict(
            connected=True,
            armed=True,
            mode="OFFBOARD",
            state_age=0.1,
            state_timeout=3.0,
        )
        self.assertTrue(ready(**base))
        for update in (
            {"connected": False},
            {"armed": False},
            {"mode": "AUTO.LOITER"},
            {"state_age": 3.1},
            {"state_age": float("inf")},
        ):
            values = dict(base)
            values.update(update)
            self.assertFalse(ready(**values))

    def test_cancelled_takeoff_has_time_for_safe_executor_recovery(self):
        self.assertEqual(MODULE.child_stop_timeout(False, 15.0), 3.0)
        self.assertEqual(MODULE.child_stop_timeout(True, 15.0), 32.0)


class FlightCommandServerContractTest(unittest.TestCase):
    def make_land_server(self, state, set_mode):
        server = MODULE.FlightCommandServer.__new__(MODULE.FlightCommandServer)
        server.state_timeout = 3.0
        server.preflight_timeout = 0.1
        server.command_timeout = 0.02
        server.land_retry_interval = 0.001
        server._condition = DummyCondition()
        received_at = time.monotonic()
        server._snapshot = lambda: (state, received_at, None, 0.0)
        server.set_mode = set_mode
        server._feedback = lambda _stage, _message: None
        server.server = SimpleNamespace(is_preempt_requested=lambda: False)
        server.FlightCommandFeedback = SimpleNamespace(REQUESTING_LAND=5)
        server.FlightCommandResult = SimpleNamespace(
            LINK_UNAVAILABLE=3,
            STATE_STALE=4,
            STATE_REJECTED=5,
            LAND_FAILED=7,
            PREEMPTED=8,
        )
        server.rospy = SimpleNamespace(
            ROSException=FakeRosException,
            ServiceException=FakeServiceException,
            wait_for_service=lambda _name, timeout: timeout,
            is_shutdown=lambda: False,
            logerr_throttle=lambda *_args: None,
        )
        return server

    def test_land_requires_observed_auto_land_not_only_mode_sent(self):
        state = SimpleNamespace(connected=True, armed=True, mode="OFFBOARD")
        calls = []

        def set_mode(**kwargs):
            calls.append(kwargs)
            return SimpleNamespace(mode_sent=True)

        server = self.make_land_server(state, set_mode)
        with self.assertRaises(MODULE.FlightCommandError) as context:
            server._request_land()
        self.assertEqual(context.exception.code, server.FlightCommandResult.LAND_FAILED)
        self.assertGreater(len(calls), 0)
        self.assertTrue(all(call["custom_mode"] == "AUTO.LAND" for call in calls))

    def test_land_succeeds_after_fresh_state_confirms_auto_land(self):
        state = SimpleNamespace(connected=True, armed=True, mode="OFFBOARD")
        calls = []

        def set_mode(**kwargs):
            calls.append(kwargs)
            state.mode = "AUTO.LAND"
            return SimpleNamespace(mode_sent=True)

        server = self.make_land_server(state, set_mode)
        server._request_land()
        self.assertEqual(len(calls), 1)

    def test_server_never_force_disarms(self):
        source = inspect.getsource(MODULE)
        self.assertNotIn("CommandBool", source)
        self.assertNotIn("value=False", source)
        self.assertIn('custom_mode="AUTO.LAND"', source)

    def test_lifecycle_file_lock_rejects_concurrent_commands(self):
        first = MODULE.FlightCommandServer.__new__(MODULE.FlightCommandServer)
        second = MODULE.FlightCommandServer.__new__(MODULE.FlightCommandServer)
        with tempfile.TemporaryDirectory() as directory:
            lock_path = os.path.join(directory, "flight.lifecycle.lock")
            first.lock_path = lock_path
            second.lock_path = lock_path
            first_lock = first._acquire_lock()
            self.assertIsNotNone(first_lock)
            try:
                self.assertIsNone(second._acquire_lock())
            finally:
                first._release_lock(first_lock)
            second_lock = second._acquire_lock()
            self.assertIsNotNone(second_lock)
            second._release_lock(second_lock)

    def test_arm_command_propagates_runtime_specific_parameters(self):
        server = MODULE.FlightCommandServer.__new__(MODULE.FlightCommandServer)
        server.arm_executor = "/tmp/arm_executor.py"
        server.preflight_timeout = 5.0
        server.command_timeout = 15.0
        server.takeoff_timeout = 30.0
        server.takeoff_tolerance = 0.1
        server.takeoff_stable_time = 0.5
        server.takeoff_max_vertical_speed = 0.2
        server.takeoff_altitude_field = "auto"
        server.disarmed_prearm_mode = "AUTO.LOITER"
        server.odometry_topic = "/localization/odom"
        server.controller_node = "/se3_controller_node"
        server.attitude_setpoint_topic = "/mavros/setpoint_raw/attitude"
        server.px4_hover_thrust = 0.755
        command = server._arm_command(1.5)
        for expected in (
            "/tmp/arm_executor.py",
            "--takeoff-height",
            "1.5",
            "--takeoff-altitude-field",
            "auto",
            "--disarmed-prearm-mode",
            "AUTO.LOITER",
            "--px4-hover-thrust",
            "0.755",
        ):
            self.assertIn(expected, command)

    def test_generated_action_and_catkin_registration_are_stable(self):
        action_path = os.path.join(
            REPOSITORY,
            "planning",
            "ros_pkgs",
            "sim2real_planning_msgs",
            "action",
            "FlightCommand.action",
        )
        if not os.path.isfile(action_path):
            self.skipTest("repository planning message sources are not mounted")
        with open(action_path, "r", encoding="utf-8") as stream:
            action = stream.read()
        for contract in (
            "uint8 TAKEOFF=1",
            "uint8 LAND=2",
            "uint16 BUSY=1",
            "uint16 PREEMPTED=8",
            "uint8 REQUESTING_LAND=5",
        ):
            self.assertIn(contract, action)

        with open(
            os.path.join(
                REPOSITORY,
                "planning",
                "ros_pkgs",
                "sim2real_planning_msgs",
                "CMakeLists.txt",
            ),
            "r",
            encoding="utf-8",
        ) as stream:
            message_cmake = stream.read()
        self.assertIn("FlightCommand.action", message_cmake)

        with open(
            os.path.join(REPOSITORY, "common", "CMakeLists.txt"),
            "r",
            encoding="utf-8",
        ) as stream:
            common_cmake = stream.read()
        self.assertIn("scripts/flight_command_server.py", common_cmake)
        self.assertIn("test/test_flight_command_server.py", common_cmake)


if __name__ == "__main__":
    unittest.main()
