#!/usr/bin/env python3

import unittest

from sim2real_planner_manager.command_gate import (
    ACTIVE,
    BRAKE,
    CommandGate,
    GateConfig,
    HOLD,
    HOLDING,
    NORMAL,
    REACHED,
    StatusSnapshot,
)


class CommandGateTest(unittest.TestCase):
    def setUp(self):
        self.gate = CommandGate(
            GateConfig(
                backend_id="fast-kino",
                session_id="session-a",
                state_timeout_sec=1.0,
                status_timeout_sec=0.5,
                command_timeout_sec=0.25,
            )
        )
        self.gate.update_vehicle(True, True, "OFFBOARD", 10.0)
        self.gate.begin_goal(1)

    def status(
        self,
        *,
        trajectory_id=7,
        state=ACTIVE,
        armable=True,
        received_at=10.0,
        session_id="session-a",
        goal_id=1,
        map_ready=True,
    ):
        return StatusSnapshot(
            session_id=session_id,
            backend_id="fast-kino",
            goal_id=goal_id,
            trajectory_id=trajectory_id,
            state=state,
            odom_ready=True,
            map_ready=map_ready,
            armable=armable,
            received_at=received_at,
        )

    def command(
        self,
        *,
        trajectory_id=7,
        mode=NORMAL,
        received_at=10.1,
        session_id="session-a",
        goal_id=1,
    ):
        return self.gate.evaluate_command(
            session_id=session_id,
            backend_id="fast-kino",
            goal_id=goal_id,
            trajectory_id=trajectory_id,
            mode=mode,
            values_finite=True,
            shape_valid=True,
            received_at=received_at,
            header_age_sec=0.01,
        )

    def test_first_normal_command_opens_gate(self):
        self.assertTrue(self.gate.update_status(self.status()).accepted)
        decision = self.command()
        self.assertTrue(decision.accepted)
        self.assertTrue(decision.opened_gate)
        self.assertTrue(self.gate.is_open)

    def test_brake_cannot_open_gate(self):
        self.gate.update_status(
            self.status(state=HOLDING, armable=False)
        )
        decision = self.command(mode=BRAKE)
        self.assertFalse(decision.accepted)
        self.assertFalse(self.gate.is_open)

    def test_authorized_goal_accepts_incrementing_brake_trajectory(self):
        self.gate.update_status(self.status())
        self.assertTrue(self.command().accepted)
        self.gate.update_status(
            self.status(
                trajectory_id=8, state=HOLDING, armable=False, received_at=10.2
            )
        )
        decision = self.command(
            trajectory_id=8, mode=BRAKE, received_at=10.21
        )
        self.assertTrue(decision.accepted)
        self.assertFalse(decision.opened_gate)

    def test_reached_allows_hold_but_not_normal_or_brake(self):
        self.gate.update_status(self.status())
        self.assertTrue(self.command().accepted)
        self.gate.update_status(
            self.status(
                trajectory_id=8, state=REACHED, armable=False, received_at=10.2
            )
        )
        self.assertTrue(
            self.command(
                trajectory_id=8, mode=HOLD, received_at=10.21
            ).accepted
        )
        self.assertFalse(
            self.command(
                trajectory_id=8, mode=BRAKE, received_at=10.22
            ).accepted
        )

    def test_new_goal_invalidates_old_status_and_commands(self):
        self.gate.update_status(self.status())
        self.assertTrue(self.command().accepted)
        self.gate.begin_goal(2)
        self.assertFalse(self.gate.is_open)
        self.assertFalse(self.command(goal_id=1).accepted)
        self.assertFalse(self.gate.update_status(self.status(goal_id=1)).accepted)

    def test_leaving_offboard_closes_gate(self):
        self.gate.update_status(self.status())
        self.assertTrue(self.command().accepted)
        self.gate.update_vehicle(True, True, "MANUAL", 10.2)
        self.assertFalse(self.gate.is_open)
        self.assertFalse(self.command(received_at=10.21).accepted)
        self.gate.update_vehicle(True, True, "OFFBOARD", 10.3)
        self.assertFalse(self.command(received_at=10.31).accepted)
        self.assertTrue(self.gate.is_revoked)

    def test_map_failure_revokes_goal_until_a_new_goal(self):
        self.gate.update_status(self.status())
        self.assertTrue(self.command().accepted)
        failed = self.status(received_at=10.2, map_ready=False)
        self.assertTrue(self.gate.update_status(failed).accepted)
        self.assertTrue(self.gate.is_revoked)
        self.assertFalse(self.command(received_at=10.21).accepted)

    def test_stale_status_and_timestamp_are_rejected(self):
        self.gate.update_status(self.status(received_at=10.0))
        self.assertFalse(self.command(received_at=10.6).accepted)
        self.gate.update_status(self.status(received_at=10.7))
        decision = self.gate.evaluate_command(
            session_id="session-a",
            backend_id="fast-kino",
            goal_id=1,
            trajectory_id=7,
            mode=NORMAL,
            values_finite=True,
            shape_valid=True,
            received_at=10.71,
            header_age_sec=0.3,
        )
        self.assertFalse(decision.accepted)

    def test_wrong_session_and_trajectory_rollback_are_rejected(self):
        self.assertFalse(
            self.gate.update_status(
                self.status(session_id="old-session")
            ).accepted
        )
        self.gate.update_status(self.status())
        self.assertTrue(self.command().accepted)
        rollback = self.status(trajectory_id=6, received_at=10.2)
        self.assertFalse(self.gate.update_status(rollback).accepted)

    def test_replacement_normal_trajectory_requires_armable(self):
        self.gate.update_status(self.status())
        self.assertTrue(self.command().accepted)
        self.gate.update_status(
            self.status(trajectory_id=8, armable=False, received_at=10.2)
        )
        self.assertFalse(
            self.command(trajectory_id=8, received_at=10.21).accepted
        )

    def test_default_state_timeout_tolerates_one_hz_mavros_jitter(self):
        gate = CommandGate(
            GateConfig(backend_id="diff", session_id="session-b")
        )
        self.assertEqual(gate.config.state_timeout_sec, 3.0)
        gate.update_vehicle(True, True, "OFFBOARD", 20.0)
        gate.begin_goal(1)
        self.assertTrue(gate.vehicle_ready(21.1))


if __name__ == "__main__":
    unittest.main()
