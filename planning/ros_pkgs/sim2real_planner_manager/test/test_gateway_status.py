#!/usr/bin/env python3

from collections import deque
import copy
import importlib.util
import os
from types import SimpleNamespace
import threading
import unittest

from sim2real_planner_manager.command_gate import CommandGate, GateConfig


SCRIPT_PATH = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "scripts", "planner_gateway.py")
)
SPEC = importlib.util.spec_from_file_location("tested_planner_gateway", SCRIPT_PATH)
GATEWAY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GATEWAY)


class FakeStamp:
    def __init__(self, seconds=0.0):
        self.seconds = float(seconds)

    def is_zero(self):
        return self.seconds == 0.0

    def to_sec(self):
        return self.seconds

    def __sub__(self, other):
        return SimpleNamespace(to_sec=lambda: self.seconds - other.seconds)


class FakeTime:
    now_seconds = 10.0

    def __new__(cls, seconds=0.0):
        return FakeStamp(seconds)

    @classmethod
    def now(cls):
        return FakeStamp(cls.now_seconds)


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(copy.deepcopy(message))


class PlannerGatewayCommandClockTest(unittest.TestCase):
    def setUp(self):
        self.original_rospy = GATEWAY.rospy
        self.original_now_seconds = FakeTime.now_seconds
        GATEWAY.rospy = SimpleNamespace(Time=FakeTime)
        self.gateway = GATEWAY.PlannerGateway.__new__(GATEWAY.PlannerGateway)
        self.gateway._gate = SimpleNamespace(is_open=True)
        self.gateway._command_timeout = 0.08

    def tearDown(self):
        FakeTime.now_seconds = self.original_now_seconds
        GATEWAY.rospy = self.original_rospy

    def test_slow_simulation_uses_ros_clock_for_command_freshness(self):
        self.gateway._runtime_mode = "simulation"
        self.gateway._last_command_stream_at = 155.274
        FakeTime.now_seconds = 155.334

        # The host advanced by 137 ms, but the simulated vehicle advanced by
        # only 60 ms. This reproduces the cross-machine false timeout.
        stream_now = self.gateway._rate_now(1000.137)
        self.assertAlmostEqual(stream_now, 155.334)
        self.assertFalse(self.gateway._command_stream_timed_out(stream_now))

        FakeTime.now_seconds = 155.364
        stream_now = self.gateway._rate_now(1000.167)
        self.assertTrue(self.gateway._command_stream_timed_out(stream_now))

    def test_real_runtime_keeps_monotonic_command_timeout(self):
        self.gateway._runtime_mode = "real"
        self.gateway._last_command_stream_at = 1000.0
        FakeTime.now_seconds = 155.280

        stream_now = self.gateway._rate_now(1000.081)
        self.assertAlmostEqual(stream_now, 1000.081)
        self.assertTrue(self.gateway._command_stream_timed_out(stream_now))


def vector(x, y, z):
    return SimpleNamespace(x=float(x), y=float(y), z=float(z))


def controller_point():
    return SimpleNamespace(
        transforms=[
            SimpleNamespace(
                translation=vector(9.0, 8.0, 7.0),
                rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        ],
        velocities=[
            SimpleNamespace(
                linear=vector(1.0, 2.0, 3.0),
                angular=vector(4.0, 5.0, 6.0),
            )
        ],
        accelerations=[
            SimpleNamespace(
                linear=vector(7.0, 8.0, 9.0),
                angular=vector(10.0, 11.0, 12.0),
            )
        ],
    )


def measured_pose():
    return SimpleNamespace(
        position=vector(1.0, 2.0, 3.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.5, w=0.8660254),
    )


class FakeTrajectory:
    def __init__(self):
        self.header = SimpleNamespace(seq=0, stamp=None, frame_id="")
        self.points = []


class HandoffHoldTest(unittest.TestCase):
    def setUp(self):
        self.original_rospy = GATEWAY.rospy
        self.original_trajectory = GATEWAY.MultiDOFJointTrajectory
        GATEWAY.rospy = SimpleNamespace(
            Time=SimpleNamespace(now=lambda: FakeStamp(12.0))
        )
        GATEWAY.MultiDOFJointTrajectory = FakeTrajectory

    def tearDown(self):
        GATEWAY.rospy = self.original_rospy
        GATEWAY.MultiDOFJointTrajectory = self.original_trajectory

    def test_stationary_point_uses_measured_pose_and_zeroes_all_motion(self):
        source = controller_point()
        result = GATEWAY.stationary_point_at_pose(source, measured_pose())
        self.assertEqual(
            (
                result.transforms[0].translation.x,
                result.transforms[0].translation.y,
                result.transforms[0].translation.z,
            ),
            (1.0, 2.0, 3.0),
        )
        for twist in result.velocities + result.accelerations:
            self.assertEqual(
                (
                    twist.linear.x,
                    twist.linear.y,
                    twist.linear.z,
                    twist.angular.x,
                    twist.angular.y,
                    twist.angular.z,
                ),
                (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            )
        self.assertEqual(source.velocities[0].linear.x, 1.0)

    def test_closed_authorized_handoff_publishes_measured_pose_hold(self):
        gateway = GATEWAY.PlannerGateway.__new__(GATEWAY.PlannerGateway)
        gateway._lock = threading.RLock()
        gateway._goal_id = 2
        gateway._goal_frame = "world"
        gateway._gate = SimpleNamespace(
            is_open=False,
            is_revoked=False,
            vehicle_ready=lambda _now: True,
        )
        gateway._last_output_point = controller_point()
        gateway._last_odom_pose = measured_pose()
        gateway._last_odom_at = 19.95
        gateway._handoff_odom_timeout = 0.2
        gateway._transition_hold_pose = None
        gateway._cancel_hold_active = False
        gateway._handoff_hold_sequence = 0
        gateway._output_pub = RecordingPublisher()
        gateway._now_monotonic = lambda: 20.0

        gateway._handoff_hold_callback(None)

        self.assertEqual(len(gateway._output_pub.messages), 1)
        output = gateway._output_pub.messages[0]
        self.assertEqual(output.header.frame_id, "world")
        self.assertEqual(output.header.seq, 1)
        self.assertEqual(output.points[0].velocities[0].linear.x, 0.0)
        self.assertEqual(output.points[0].transforms[0].translation.x, 1.0)

    def test_revoked_fault_does_not_publish_transition_hold(self):
        gateway = GATEWAY.PlannerGateway.__new__(GATEWAY.PlannerGateway)
        gateway._lock = threading.RLock()
        gateway._goal_id = 2
        gateway._goal_frame = "world"
        gateway._gate = SimpleNamespace(
            is_open=False,
            is_revoked=True,
            vehicle_ready=lambda _now: True,
        )
        gateway._last_output_point = controller_point()
        gateway._last_odom_pose = measured_pose()
        gateway._last_odom_at = 19.95
        gateway._handoff_odom_timeout = 0.2
        gateway._transition_hold_pose = None
        gateway._cancel_hold_active = False
        gateway._handoff_hold_sequence = 0
        gateway._output_pub = RecordingPublisher()
        gateway._now_monotonic = lambda: 20.0

        gateway._handoff_hold_callback(None)

        self.assertEqual(gateway._output_pub.messages, [])

        gateway._cancel_hold_active = True
        gateway._handoff_hold_callback(None)
        self.assertEqual(len(gateway._output_pub.messages), 1)


class PlannerGatewayRevokedStatusTest(unittest.TestCase):
    def setUp(self):
        self.original_rospy = GATEWAY.rospy
        GATEWAY.rospy = SimpleNamespace(
            Time=FakeTime,
            logwarn_throttle=lambda *_args, **_kwargs: None,
            logerr_throttle=lambda *_args, **_kwargs: None,
        )

        gateway = GATEWAY.PlannerGateway.__new__(GATEWAY.PlannerGateway)
        gateway._lock = threading.RLock()
        gateway._planner_id = "diff"
        gateway._session_id = "session-a"
        gateway._goal_frame = "world"
        gateway._goal_id = 1
        gateway._status_timeout = 1.0
        gateway._last_backend_status = None
        gateway._last_backend_status_at = 0.0
        gateway._last_backend_status_stamp = FakeStamp()
        gateway._last_backend_status_seq = -1
        gateway._status_receipts = deque(maxlen=16)
        gateway._fault_reported = False
        gateway._status_pub = RecordingPublisher()
        gateway._gate = CommandGate(
            GateConfig(backend_id="diff", session_id="session-a")
        )
        gateway._gate.begin_goal(1)
        gateway._gate.cancel_goal()
        gateway._trusted_backend_message = lambda *_args: True
        gateway._rate_sample_time = lambda receipt, _stamp: receipt
        gateway._update_ready_time = lambda _now: None
        gateway._now_monotonic = lambda: 20.0
        self.gateway = gateway

    def tearDown(self):
        GATEWAY.rospy = self.original_rospy

    @staticmethod
    def status(state, reason, *, stamp=9.9, sequence=1):
        return SimpleNamespace(
            header=SimpleNamespace(
                stamp=FakeStamp(stamp), frame_id="world", seq=sequence
            ),
            session_id="session-a",
            backend_id="diff",
            goal_id=1,
            trajectory_id=7,
            state=state,
            odom_ready=True,
            map_ready=True,
            armable=False,
            reason=reason,
        )

    def test_backend_fault_remains_fault_after_goal_authorization_revoked(self):
        self.gateway._status_callback(
            self.status(
                GATEWAY.PlannerStatus.FAULT,
                "backend heartbeat timed out",
                stamp=9.8,
                sequence=1,
            )
        )
        self.gateway._status_callback(
            self.status(
                GATEWAY.PlannerStatus.FAULT,
                "backend heartbeat still timed out",
                stamp=9.9,
                sequence=2,
            )
        )

        published = self.gateway._status_pub.messages[-2:]
        self.assertEqual(
            [message.state for message in published],
            [GATEWAY.PlannerStatus.FAULT, GATEWAY.PlannerStatus.FAULT],
        )
        self.assertEqual(
            published[-1].reason, "backend heartbeat still timed out"
        )
        self.assertTrue(all(not message.armable for message in published))

    def test_normal_cancel_still_projects_healthy_backend_status_to_holding(self):
        self.gateway._status_callback(
            self.status(GATEWAY.PlannerStatus.HOLDING, "reset complete")
        )

        published = self.gateway._status_pub.messages[-1]
        self.assertEqual(published.state, GATEWAY.PlannerStatus.HOLDING)
        self.assertEqual(
            published.reason,
            "goal authorization is revoked; submit a new goal",
        )
        self.assertFalse(published.armable)


if __name__ == "__main__":
    unittest.main()
