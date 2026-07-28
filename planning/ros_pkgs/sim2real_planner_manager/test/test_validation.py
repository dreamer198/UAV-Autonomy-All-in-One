#!/usr/bin/env python3

from types import SimpleNamespace
import unittest

from sim2real_planner_manager.validation import (
    backend_status_allows_new_goal,
    status_order_is_newer,
    validate_command_mode,
)


def vector(x=0.0, y=0.0, z=0.0):
    return SimpleNamespace(x=x, y=y, z=z)


def command_point(position=None, velocity=None, acceleration=None):
    def twist(linear):
        return SimpleNamespace(linear=linear, angular=vector())

    return SimpleNamespace(
        transforms=[
            SimpleNamespace(translation=position or vector())
        ],
        velocities=[
            twist(velocity or vector())
        ],
        accelerations=(
            []
            if acceleration is None
            else [twist(acceleration)]
        ),
    )


class CommandLimitValidationTest(unittest.TestCase):
    def test_status_order_accepts_same_stamp_only_when_sequence_advances(self):
        self.assertTrue(status_order_is_newer(10.0, 8, 10.0, 7))
        self.assertFalse(status_order_is_newer(10.0, 7, 10.0, 7))
        self.assertFalse(status_order_is_newer(9.9, 9, 10.0, 7))
        self.assertTrue(status_order_is_newer(10.1, 0, 10.0, 7))

    def test_hold_requires_zero_velocity_and_acceleration(self):
        valid, reason = validate_command_mode(
            1,
            command_point(
                velocity=vector(0.01, 0.0, 0.0),
                acceleration=vector(),
            ),
        )
        self.assertFalse(valid)
        self.assertIn("zero", reason)
        valid, reason = validate_command_mode(
            1,
            command_point(velocity=vector(), acceleration=vector()),
        )
        self.assertTrue(valid, reason)

    def test_active_reached_holding_and_fault_can_accept_recovery_goal(self):
        allowed = {1, 2, 3, 4, 5, 6}
        for state in (3, 4, 5, 6):
            status = SimpleNamespace(
                state=state, odom_ready=True, map_ready=True
            )
            self.assertTrue(
                backend_status_allows_new_goal(
                    status,
                    received_at=9.9,
                    now=10.0,
                    timeout=0.5,
                    allowed_states=allowed,
                )
            )
        self.assertFalse(
            backend_status_allows_new_goal(
                SimpleNamespace(
                    state=0, odom_ready=True, map_ready=True
                ),
                received_at=9.9,
                now=10.0,
                timeout=0.5,
                allowed_states=allowed,
            )
        )
        self.assertFalse(
            backend_status_allows_new_goal(
                SimpleNamespace(
                    state=3, odom_ready=True, map_ready=True
                ),
                received_at=9.0,
                now=10.0,
                timeout=0.5,
                allowed_states=allowed,
            )
        )


if __name__ == "__main__":
    unittest.main()
