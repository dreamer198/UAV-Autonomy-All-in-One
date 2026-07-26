#!/usr/bin/env python3

import importlib.util
import inspect
import json
import math
import os
import tempfile
import threading
import time
import unittest
from types import SimpleNamespace


SCRIPT_PATH = os.path.join(
    os.path.dirname(__file__), "..", "scripts", "waypoint_mission.py"
)
SPEC = importlib.util.spec_from_file_location("waypoint_mission", SCRIPT_PATH)
MISSION = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MISSION)


class WaypointMissionConfigTest(unittest.TestCase):
    def write_config(self, payload):
        handle = tempfile.NamedTemporaryFile(
            mode="w", suffix=".json", encoding="utf-8", delete=False
        )
        self.addCleanup(lambda: os.path.exists(handle.name) and os.unlink(handle.name))
        with handle:
            json.dump(payload, handle)
        return handle.name

    def test_runtime_waits_do_not_depend_on_ros_sim_time(self):
        with open(SCRIPT_PATH, "r", encoding="utf-8") as stream:
            source = stream.read()
        self.assertNotIn("rospy.Rate", source)
        self.assertNotIn("rospy.sleep", source)
        self.assertIn("time.sleep(duration)", source)

    def test_state_topic_is_configurable_for_remapped_mavros(self):
        args = MISSION._build_parser().parse_args(["mission.json"])
        self.assertEqual(args.state_topic, "/mavros/state")
        signature = inspect.signature(MISSION.WaypointMission.__init__)
        self.assertIn("state_topic", signature.parameters)

    def test_normalizes_ordered_waypoints_and_automatic_yaw(self):
        path = self.write_config(
            {
                "land_after_mission": True,
                "waypoints": [
                    {"x": 1, "y": 2, "z": 1},
                    {"x": 3, "y": 4, "z": 1.5, "yaw": 90},
                ],
            }
        )
        config = MISSION.load_mission_config(
            path,
            default_takeoff_height=1.0,
            virtual_ground=0.1,
            virtual_ceil=3.0,
        )
        self.assertEqual(config["takeoff_height"], 1.0)
        self.assertEqual(config["takeoff_settle_time"], 0.0)
        self.assertTrue(config["fly_through"])
        self.assertEqual(config["fly_through_tolerance"], 0.5)
        self.assertTrue(config["waypoints"][0]["fly_through"])
        self.assertEqual(len(config["waypoints"]), 2)
        self.assertEqual(config["planner_recovery_timeout"], 2.0)
        self.assertEqual(config["planner_retry_limit"], 3)
        self.assertAlmostEqual(config["waypoints"][0]["yaw"], 45.0)
        self.assertEqual(config["waypoints"][1]["yaw"], 90.0)

    def test_automatic_yaw_faces_the_following_segment(self):
        path = self.write_config(
            {
                "takeoff_height": 1.0,
                "waypoints": [
                    {"x": 0, "y": 0, "z": 1},
                    {"x": 0, "y": 2, "z": 1},
                    {"x": -2, "y": 2, "z": 1},
                ],
            }
        )
        config = MISSION.load_mission_config(path)
        yaws = [waypoint["yaw"] for waypoint in config["waypoints"]]
        self.assertAlmostEqual(yaws[0], 90.0)
        self.assertAlmostEqual(abs(yaws[1]), 180.0)
        self.assertAlmostEqual(abs(yaws[2]), 180.0)

    def test_explicit_yaw_overrides_automatic_yaw(self):
        path = self.write_config(
            {
                "takeoff_height": 1.0,
                "waypoints": [
                    {"x": 0, "y": 0, "z": 1, "yaw": 12},
                    {"x": 0, "y": 2, "z": 1},
                ],
            }
        )
        config = MISSION.load_mission_config(path)
        self.assertEqual(config["waypoints"][0]["yaw"], 12.0)
        self.assertAlmostEqual(config["waypoints"][1]["yaw"], 90.0)

    def test_skips_exactly_coincident_point_when_later_direction_exists(self):
        path = self.write_config(
            {
                "takeoff_height": 1.0,
                "waypoints": [
                    {"x": 0, "y": 0, "z": 1},
                    {"x": 0, "y": 0, "z": 2},
                    {"x": 2, "y": 0, "z": 2},
                ],
            }
        )
        config = MISSION.load_mission_config(path)
        self.assertEqual(
            [waypoint["yaw"] for waypoint in config["waypoints"]],
            [0.0, 0.0, 0.0],
        )

    def test_skips_near_coincident_point_below_fly_through_radius(self):
        path = self.write_config(
            {
                "takeoff_height": 1.0,
                "fly_through_tolerance": 0.5,
                "waypoints": [
                    {"x": 0, "y": 0, "z": 1},
                    {"x": 0.01, "y": 0.01, "z": 1},
                    {"x": 2, "y": 0, "z": 1},
                ],
            }
        )
        config = MISSION.load_mission_config(path)
        yaws = [waypoint["yaw"] for waypoint in config["waypoints"]]
        self.assertAlmostEqual(yaws[0], 0.0)
        self.assertAlmostEqual(
            yaws[1], math.degrees(math.atan2(-0.01, 1.99))
        )
        self.assertAlmostEqual(yaws[2], yaws[1])

    def test_near_coincident_tail_inherits_last_valid_heading(self):
        path = self.write_config(
            {
                "takeoff_height": 1.0,
                "fly_through_tolerance": 0.5,
                "waypoints": [
                    {"x": 0, "y": 0, "z": 1},
                    {"x": 2, "y": 0, "z": 1},
                    {"x": 2.01, "y": 0.01, "z": 1},
                ],
            }
        )
        config = MISSION.load_mission_config(path)
        self.assertEqual(
            [waypoint["yaw"] for waypoint in config["waypoints"]],
            [0.0, 0.0, 0.0],
        )

    def test_vertical_only_route_locks_current_heading_at_runtime(self):
        path = self.write_config(
            {
                "takeoff_height": 1.0,
                "waypoints": [
                    {"x": 0, "y": 0, "z": 1},
                    {"x": 0, "y": 0, "z": 2},
                ],
            }
        )
        config = MISSION.load_mission_config(path)
        self.assertEqual(
            [waypoint["yaw"] for waypoint in config["waypoints"]],
            [None, None],
        )
        filled = MISSION.fill_unresolved_waypoint_yaws(
            config["waypoints"], 37.0
        )
        self.assertEqual(filled, 2)
        self.assertEqual(
            [waypoint["yaw"] for waypoint in config["waypoints"]],
            [37.0, 37.0],
        )

    def test_runtime_yaw_resolution_reads_fresh_odometry_heading(self):
        yaw = math.radians(-42.0)
        odom = SimpleNamespace(
            pose=SimpleNamespace(
                pose=SimpleNamespace(
                    orientation=SimpleNamespace(
                        x=0.0,
                        y=0.0,
                        z=math.sin(yaw / 2.0),
                        w=math.cos(yaw / 2.0),
                    )
                )
            )
        )
        runner = MISSION.WaypointMission.__new__(MISSION.WaypointMission)
        runner.config = {
            "fly_through_tolerance": 0.5,
            "waypoints": [{"yaw": None}, {"yaw": None}],
        }
        runner.rospy = SimpleNamespace(loginfo=lambda *_args: None)
        runner._snapshot = lambda: (None, 0.0, odom, 0.0, False, None, None)

        ready, reason = runner._resolve_runtime_yaws()

        self.assertTrue(ready)
        self.assertEqual(reason, "")
        self.assertAlmostEqual(runner.config["waypoints"][0]["yaw"], -42.0)
        self.assertAlmostEqual(runner.config["waypoints"][1]["yaw"], -42.0)

    def test_rejects_waypoint_on_virtual_wall(self):
        path = self.write_config(
            {"takeoff_height": 1.0, "waypoints": [{"x": 0, "y": 0, "z": 3.0}]}
        )
        with self.assertRaises(MISSION.MissionConfigError):
            MISSION.load_mission_config(
                path, virtual_ground=0.1, virtual_ceil=3.0
            )

    def test_rejects_waypoint_without_obstacle_inflation_clearance(self):
        path = self.write_config(
            {"takeoff_height": 0.5, "waypoints": [{"x": 0, "y": 0, "z": 0.4}]}
        )
        with self.assertRaisesRegex(
            MISSION.MissionConfigError, "ground clearance"
        ):
            MISSION.load_mission_config(
                path,
                virtual_ground=0.1,
                virtual_ceil=3.0,
                obstacles_inflation=0.33,
            )

    def test_rejects_nonfinite_planner_vertical_fence(self):
        path = self.write_config(
            {"takeoff_height": 1.0, "waypoints": [{"x": 0, "y": 0, "z": 1.0}]}
        )
        for field, value in (
            ("virtual_ground", float("nan")),
            ("virtual_ground", float("inf")),
            ("virtual_ground", float("-inf")),
            ("virtual_ceil", float("nan")),
            ("virtual_ceil", float("inf")),
            ("virtual_ceil", float("-inf")),
        ):
            bounds = {"virtual_ground": 0.1, "virtual_ceil": 3.0}
            bounds[field] = value
            with self.subTest(field=field, value=value):
                with self.assertRaisesRegex(
                    MISSION.MissionConfigError, "{} must be finite".format(field)
                ):
                    MISSION.load_mission_config(
                        path,
                        obstacles_inflation=0.33,
                        **bounds
                    )

    def test_accepts_height_inside_inflated_vertical_clearance(self):
        path = self.write_config(
            {"takeoff_height": 0.5, "waypoints": [{"x": 0, "y": 0, "z": 0.5}]}
        )
        config = MISSION.load_mission_config(
            path,
            virtual_ground=0.1,
            virtual_ceil=3.0,
            obstacles_inflation=0.33,
        )
        self.assertEqual(config["waypoints"][0]["z"], 0.5)

    def test_rejects_nonfinite_coordinate(self):
        path = self.write_config(
            {
                "takeoff_height": 1.0,
                "waypoints": [{"x": math.nan, "y": 0, "z": 1.0}],
            }
        )
        with self.assertRaises(MISSION.MissionConfigError):
            MISSION.load_mission_config(path)

    def test_rejects_unknown_field(self):
        path = self.write_config(
            {
                "takeoff_height": 1.0,
                "waypoints": [{"x": 0, "y": 0, "z": 1.0, "yaaw": 90}],
            }
        )
        with self.assertRaises(MISSION.MissionConfigError):
            MISSION.load_mission_config(path)

    def test_rejects_non_boolean_fly_through(self):
        path = self.write_config(
            {
                "takeoff_height": 1.0,
                "fly_through": "yes",
                "waypoints": [{"x": 0, "y": 0, "z": 1.0}],
            }
        )
        with self.assertRaises(MISSION.MissionConfigError):
            MISSION.load_mission_config(path)

    def test_final_waypoint_always_stops(self):
        waypoint = {"fly_through": True}
        self.assertTrue(MISSION.waypoint_is_fly_through(waypoint, 1, 2))
        self.assertFalse(MISSION.waypoint_is_fly_through(waypoint, 2, 2))

    def test_fly_through_requires_position_and_yaw(self):
        ready = MISSION.fly_through_arrival_ready
        self.assertTrue(ready(0.49, 0.5, True))
        self.assertFalse(ready(0.51, 0.5, True))
        self.assertFalse(ready(0.49, 0.5, False))

    def test_localization_loss_lands_instead_of_requesting_loiter(self):
        recovery = MISSION.mission_failure_recovery_mode
        self.assertEqual(
            recovery(MISSION.LOCALIZATION_STALE_REASON), "AUTO.LAND"
        )
        self.assertEqual(recovery("Planner did not accept the waypoint"), "AUTO.LOITER")

    def test_localization_fault_latch_is_understood(self):
        reason = MISSION.localization_fault_reason
        self.assertEqual(reason(None), "")
        self.assertEqual(
            reason({"active": True, "reason": "FAST-LIO diverged"}),
            "FAST-LIO diverged",
        )

    def test_existing_localization_fault_reason_is_not_overwritten(self):
        stored = {"active": True, "reason": "FAST-LIO timestamp moved backwards"}
        writes = []
        runner = MISSION.WaypointMission.__new__(MISSION.WaypointMission)
        runner.lock = threading.Lock()
        runner.abort_requested = False
        runner.localization_fault_latched = False
        runner.rospy = SimpleNamespace(
            get_param=lambda *_args: stored,
            set_param=lambda *args: writes.append(args),
        )

        code, reason = runner._flight_gate()

        self.assertEqual(code, MISSION.EXIT_MISSION_FAILED)
        self.assertEqual(reason, MISSION.LOCALIZATION_STALE_REASON)
        self.assertTrue(runner.localization_fault_latched)
        self.assertEqual(writes, [])
        self.assertEqual(
            stored["reason"], "FAST-LIO timestamp moved backwards"
        )

    def test_initial_wait_preserves_latched_localization_failure(self):
        runner = MISSION.WaypointMission.__new__(MISSION.WaypointMission)
        runner.config = {"planner_accept_timeout": 1.0}
        runner.rospy = SimpleNamespace(is_shutdown=lambda: False)
        runner.goal_pub = SimpleNamespace(get_num_connections=lambda: 0)
        runner._flight_gate = lambda: (
            MISSION.EXIT_MISSION_FAILED,
            MISSION.LOCALIZATION_STALE_REASON,
        )
        runner._wall_wait = lambda: self.fail(
            "latched localization failure must not wait for topic timeout"
        )

        code, reason = runner._wait_for_initial_state()

        self.assertEqual(code, MISSION.EXIT_MISSION_FAILED)
        self.assertEqual(reason, MISSION.LOCALIZATION_STALE_REASON)

    def test_temporary_emergency_stop_is_cleared_by_recovery_trajectory(self):
        runner = MISSION.WaypointMission.__new__(MISSION.WaypointMission)
        runner.lock = threading.Lock()
        runner.current_goal_stamp = 123
        runner.current_requested_goal_position = (1.0, 2.0, 3.0)
        runner.current_goal_position = (1.0, 2.0, 3.0)
        runner.current_plan_accepted = True
        runner.current_planner_stopped_at = None

        stop = SimpleNamespace(
            goal_stamp=123,
            goal_position=(1.0, 2.0, 3.0),
            armable=False,
            traj_id=2,
        )
        runner._trajectory_callback(stop)
        self.assertIsNotNone(runner.current_planner_stopped_at)

        recovered = SimpleNamespace(
            goal_stamp=123,
            goal_position=(1.0, 2.0, 3.0),
            armable=True,
            traj_id=3,
        )
        runner._trajectory_callback(recovered)
        self.assertTrue(runner.current_plan_accepted)
        self.assertIsNone(runner.current_planner_stopped_at)

    def test_initial_emergency_stop_starts_bounded_retry_window(self):
        runner = MISSION.WaypointMission.__new__(MISSION.WaypointMission)
        runner.lock = threading.Lock()
        runner.current_goal_stamp = 123
        runner.current_requested_goal_position = (1.0, 2.0, 3.0)
        runner.current_goal_position = (1.0, 2.0, 3.0)
        runner.current_plan_accepted = False
        runner.current_planner_stopped_at = None

        stop = SimpleNamespace(
            goal_stamp=123,
            goal_position=(1.0, 2.0, 3.0),
            armable=False,
            traj_id=1,
        )
        runner._trajectory_callback(stop)

        self.assertFalse(runner.current_plan_accepted)
        self.assertIsNotNone(runner.current_planner_stopped_at)

    def test_repeated_stop_messages_do_not_restart_recovery_window(self):
        runner = MISSION.WaypointMission.__new__(MISSION.WaypointMission)
        runner.lock = threading.Lock()
        runner.current_goal_stamp = 123
        runner.current_requested_goal_position = (1.0, 2.0, 3.0)
        runner.current_goal_position = (1.0, 2.0, 3.0)
        runner.current_plan_accepted = True
        runner.current_planner_stopped_at = None
        stop = SimpleNamespace(
            goal_stamp=123,
            goal_position=(1.0, 2.0, 3.0),
            armable=False,
            traj_id=2,
        )

        runner._trajectory_callback(stop)
        first_stopped_at = runner.current_planner_stopped_at
        time.sleep(0.001)
        runner._trajectory_callback(stop)

        self.assertEqual(runner.current_planner_stopped_at, first_stopped_at)

    def test_collision_free_fallback_becomes_active_mission_goal(self):
        warnings = []
        runner = MISSION.WaypointMission.__new__(MISSION.WaypointMission)
        runner.lock = threading.Lock()
        runner.rospy = SimpleNamespace(
            logwarn=lambda message, *args: warnings.append(message % args)
        )
        runner.current_goal_stamp = 123
        runner.current_requested_goal_position = (1.0, 2.0, 3.0)
        runner.current_goal_position = (1.0, 2.0, 3.0)
        runner.current_plan_accepted = False
        runner.current_planner_stopped_at = None

        fallback = SimpleNamespace(
            goal_stamp=123,
            goal_position=(0.7, 2.2, 3.1),
            armable=True,
            traj_id=2,
        )
        runner._trajectory_callback(fallback)

        self.assertTrue(runner.current_plan_accepted)
        self.assertEqual(runner.current_goal_position, (0.7, 2.2, 3.1))
        self.assertIsNone(runner.current_planner_stopped_at)
        self.assertEqual(len(warnings), 1)
        self.assertIn("accepting the fallback", warnings[0])

    def test_emergency_after_fallback_still_starts_recovery_window(self):
        runner = MISSION.WaypointMission.__new__(MISSION.WaypointMission)
        runner.lock = threading.Lock()
        runner.current_goal_stamp = 123
        runner.current_requested_goal_position = (1.0, 2.0, 3.0)
        runner.current_goal_position = (0.7, 2.2, 3.1)
        runner.current_plan_accepted = True
        runner.current_planner_stopped_at = None

        stop = SimpleNamespace(
            goal_stamp=123,
            # An emergency trajectory may carry either the requested or the
            # already substituted goal; both belong to the same stamped goal.
            goal_position=(1.0, 2.0, 3.0),
            armable=False,
            traj_id=3,
        )
        runner._trajectory_callback(stop)

        self.assertIsNotNone(runner.current_planner_stopped_at)

    def test_emergency_stop_only_fails_after_recovery_timeout(self):
        runner = MISSION.WaypointMission.__new__(MISSION.WaypointMission)
        runner.config = {"planner_recovery_timeout": 2.0}
        runner.rospy = SimpleNamespace(logwarn_throttle=lambda *args: None)

        self.assertFalse(
            runner._planner_stop_is_persistent(time.monotonic() - 1.0)
        )
        self.assertTrue(
            runner._planner_stop_is_persistent(time.monotonic() - 3.0)
        )

    def test_retry_republishes_active_fallback_with_a_fresh_stamp(self):
        published = []
        warnings = []
        runner = MISSION.WaypointMission.__new__(MISSION.WaypointMission)
        runner.lock = threading.Lock()
        runner.config = {"planner_retry_limit": 3}
        runner.rospy = SimpleNamespace(
            Time=SimpleNamespace(now=lambda: 456),
            logwarn=lambda message, *args: warnings.append(message % args),
        )
        runner.goal_pub = SimpleNamespace(
            publish=lambda message: published.append(message)
        )
        runner.current_requested_goal_position = (1.0, 2.0, 3.0)
        runner.current_goal_position = (0.7, 2.2, 3.1)
        runner.current_goal_stamp = 123
        runner.current_goal_message = SimpleNamespace(
            header=SimpleNamespace(stamp=123),
            pose=SimpleNamespace(
                position=SimpleNamespace(x=1.0, y=2.0, z=3.0)
            ),
        )
        runner.current_plan_accepted = True
        runner.current_planner_stopped_at = time.monotonic() - 3.0
        runner.current_planner_retry_count = 0

        retried, reason = runner._retry_active_goal()

        self.assertTrue(retried)
        self.assertEqual(reason, "")
        self.assertEqual(runner.current_goal_stamp, 456)
        self.assertEqual(runner.current_planner_retry_count, 1)
        self.assertFalse(runner.current_plan_accepted)
        self.assertIsNone(runner.current_planner_stopped_at)
        self.assertEqual(len(published), 1)
        self.assertEqual(published[0].pose.position.x, 0.7)
        self.assertEqual(published[0].pose.position.y, 2.2)
        self.assertEqual(published[0].pose.position.z, 3.1)
        self.assertIn("retry 1/3", warnings[0])

    def test_retry_limit_prevents_an_infinite_mission_loop(self):
        runner = MISSION.WaypointMission.__new__(MISSION.WaypointMission)
        runner.lock = threading.Lock()
        runner.config = {"planner_retry_limit": 2}
        runner.current_planner_retry_count = 2
        runner.current_goal_message = object()
        runner.current_goal_position = (1.0, 2.0, 3.0)

        retried, reason = runner._retry_active_goal()

        self.assertFalse(retried)
        self.assertIn("exhausted 2 retries", reason)


if __name__ == "__main__":
    unittest.main()
