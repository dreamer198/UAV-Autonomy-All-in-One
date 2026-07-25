#!/usr/bin/env python3

import importlib.util
import math
import os
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

    def test_rejects_nonfinite_odometry(self):
        reason = GUARD.odometry_sanity_reason(
            odometry(position=(math.nan, 0.0, 0.0)),
            None,
            max_speed=3.0,
            max_jump=2.0,
        )
        self.assertIn("non-finite", reason)

    def test_rejects_fast_or_discontinuous_odometry(self):
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


if __name__ == "__main__":
    unittest.main()
