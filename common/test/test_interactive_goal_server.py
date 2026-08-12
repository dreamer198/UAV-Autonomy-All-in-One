#!/usr/bin/env python3

import importlib.util
import math
import os
import unittest
from types import SimpleNamespace


SCRIPT = os.path.join(
    os.path.dirname(__file__), "..", "scripts", "interactive_goal_server.py"
)
SPEC = importlib.util.spec_from_file_location("interactive_goal_server", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def request(x=1.0, y=2.0, z=1.5, takeoff=1.5, frame="world"):
    return SimpleNamespace(
        target=SimpleNamespace(
            header=SimpleNamespace(frame_id=frame),
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=y, z=z),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        ),
        takeoff_height=takeoff,
    )


class InteractiveGoalSafetyTest(unittest.TestCase):
    def test_arm_cancel_allows_complete_safe_recovery_window(self):
        self.assertEqual(MODULE.child_stop_timeout(False, 15.0), 3.0)
        self.assertEqual(MODULE.child_stop_timeout(True, 15.0), 32.0)

    def test_valid_request_and_yaw(self):
        values, yaw = MODULE.validate_goal_request(request())
        self.assertEqual(values, (1.0, 2.0, 1.5))
        self.assertAlmostEqual(yaw, 0.0)

        quarter_turn = request()
        quarter_turn.target.pose.orientation.z = math.sin(math.pi / 4.0)
        quarter_turn.target.pose.orientation.w = math.cos(math.pi / 4.0)
        _, yaw = MODULE.validate_goal_request(quarter_turn)
        self.assertAlmostEqual(yaw, 90.0)

    def test_invalid_request_rejected_before_flight(self):
        for bad in (
            request(x=float("nan")),
            request(takeoff=0.0),
            request(takeoff=0.49),
            request(takeoff=2.51),
            request(takeoff=float("inf")),
            request(frame="map"),
        ):
            with self.assertRaises(ValueError):
                MODULE.validate_goal_request(bad)

    def test_takeoff_height_bounds_are_accepted(self):
        for height in (
            MODULE.MIN_TAKEOFF_HEIGHT,
            MODULE.MAX_TAKEOFF_HEIGHT,
        ):
            values, _yaw = MODULE.validate_goal_request(
                request(takeoff=height)
            )
            self.assertEqual(values, (1.0, 2.0, 1.5))

    def test_disarmed_requires_fresh_on_ground_confirmation(self):
        base = dict(
            connected=True,
            armed=False,
            mode="AUTO.LOITER",
            state_age=0.1,
            landed_state=MODULE.ON_GROUND,
            extended_state_age=0.1,
            state_timeout=3.0,
            auto_arm_if_grounded=True,
        )
        self.assertEqual(
            MODULE.vehicle_request_kind(**base), "disarmed_ground"
        )
        for update in (
            {"auto_arm_if_grounded": False},
            {"landed_state": 2},
            {"extended_state_age": 4.0},
            {"connected": False},
        ):
            values = dict(base)
            values.update(update)
            with self.assertRaises(ValueError):
                MODULE.vehicle_request_kind(**values)

    def test_armed_vehicle_must_already_be_offboard(self):
        base = dict(
            connected=True,
            armed=True,
            mode="OFFBOARD",
            state_age=0.1,
            landed_state=0,
            extended_state_age=float("inf"),
            state_timeout=3.0,
            auto_arm_if_grounded=False,
        )
        self.assertEqual(
            MODULE.vehicle_request_kind(**base), "airborne_offboard"
        )
        base["mode"] = "POSCTL"
        with self.assertRaises(ValueError):
            MODULE.vehicle_request_kind(**base)


if __name__ == "__main__":
    unittest.main()
