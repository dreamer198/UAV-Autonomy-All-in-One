#!/usr/bin/env python3

import importlib.util
import math
import os
import threading
import time
import unittest
from types import SimpleNamespace


SCRIPT_PATH = os.path.join(
    os.path.dirname(__file__), "..", "scripts", "localization_guard.py"
)
SPEC = importlib.util.spec_from_file_location("localization_guard", SCRIPT_PATH)
GUARD = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GUARD)


def odometry(position=(0.0, 0.0, 0.0), velocity=(0.0, 0.0, 0.0)):
    return SimpleNamespace(
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(
                    x=position[0], y=position[1], z=position[2]
                ),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        ),
        twist=SimpleNamespace(
            twist=SimpleNamespace(
                linear=SimpleNamespace(
                    x=velocity[0], y=velocity[1], z=velocity[2]
                )
            )
        ),
    )


class LocalizationGuardTest(unittest.TestCase):
    def test_reads_boolean_string_and_structured_fault_latches(self):
        reason = GUARD.localization_fault_reason
        self.assertEqual(reason(""), "")
        self.assertEqual(reason(False), "")
        self.assertEqual(reason("lost lidar"), "lost lidar")
        self.assertEqual(
            reason({"active": True, "reason": "odom jumped"}), "odom jumped"
        )
        self.assertEqual(reason({"active": False, "reason": "old"}), "")

    def test_accepts_finite_stationary_odometry(self):
        self.assertEqual(
            GUARD.odometry_sanity_reason(
                odometry(), None, max_speed=3.0, max_jump=2.0
            ),
            "",
        )

    def test_default_policy_accepts_any_finite_vehicle_speed(self):
        self.assertEqual(
            GUARD.odometry_sanity_reason(
                odometry(velocity=(30.0, -20.0, 10.0)),
                None,
                max_speed=0.0,
                max_jump=2.0,
            ),
            "",
        )

    def test_rejects_nonfinite_odometry(self):
        reason = GUARD.odometry_sanity_reason(
            odometry(position=(math.nan, 0.0, 0.0)),
            None,
            max_speed=3.0,
            max_jump=2.0,
        )
        self.assertIn("non-finite", reason)

    def test_optional_speed_ceiling_and_position_jump_remain_configurable(self):
        speed_reason = GUARD.odometry_sanity_reason(
            odometry(velocity=(4.0, 0.0, 0.0)),
            None,
            max_speed=3.0,
            max_jump=2.0,
        )
        jump_reason = GUARD.odometry_sanity_reason(
            odometry(position=(10.0, 0.0, 0.0)),
            (0.0, 0.0, 0.0),
            max_speed=3.0,
            max_jump=2.0,
        )
        self.assertIn("speed", speed_reason)
        self.assertIn("jumped", jump_reason)

    def test_timestamp_must_be_fresh_and_monotonically_advance(self):
        reason = GUARD.odometry_timestamp_reason
        self.assertEqual(reason(10.0, 10.1, 9.9, 0.5, 0.1), "")
        self.assertEqual(
            reason(10.0, 10.1, 10.0, 0.5, 0.1),
            GUARD.TIMESTAMP_NOT_ADVANCING,
        )
        self.assertIn("backwards", reason(9.9, 10.0, 10.0, 0.5, 0.1))
        self.assertIn("old", reason(9.0, 10.0, None, 0.5, 0.1))
        self.assertIn("future", reason(10.2, 10.0, None, 0.5, 0.1))

    def test_zero_sim_time_waits_for_clock_instead_of_latching(self):
        self.assertEqual(
            GUARD.odometry_timestamp_reason(
                0.0, 0.0, None, max_age=0.5, future_tolerance=0.1
            ),
            GUARD.TIMESTAMP_NOT_ADVANCING,
        )

    def test_never_receiving_valid_odometry_times_out(self):
        guard = GUARD.LocalizationGuard.__new__(GUARD.LocalizationGuard)
        guard.lock = threading.Lock()
        guard.last_healthy_odom_at = None
        guard.fault_reason = ""
        guard.started_at = time.monotonic() - 6.0
        guard.startup_timeout = 5.0
        guard.odom_timeout = 0.5
        faults = []
        guard._latch_fault = faults.append
        guard._request_land_if_needed = lambda _now: None

        guard._timer_callback(None)

        self.assertEqual(len(faults), 1)
        self.assertIn("no valid localization", faults[0])


if __name__ == "__main__":
    unittest.main()
