#!/usr/bin/env python3
"""Validate and execute an ordered Diff-Planner waypoint mission."""

import argparse
import json
import math
import os
import signal
import sys
import threading
import time


EXIT_SUCCESS = 0
EXIT_MANUAL_TAKEOVER = 10
EXIT_MISSION_FAILED = 20
LOCALIZATION_STALE_REASON = "localization odometry is unavailable or stale"
LOCALIZATION_FAULT_PARAM = "/sim2real/localization_fault"

TOP_LEVEL_KEYS = {
    "takeoff_height",
    "takeoff_settle_time",
    "land_after_mission",
    "fly_through",
    "fly_through_tolerance",
    "position_tolerance",
    "yaw_tolerance_deg",
    "velocity_tolerance",
    "hold_time",
    "waypoint_timeout",
    "planner_accept_timeout",
    "planner_recovery_timeout",
    "planner_retry_limit",
    "state_timeout",
    "odom_timeout",
    "waypoints",
}
WAYPOINT_KEYS = {
    "x",
    "y",
    "z",
    "yaw",
    "position_tolerance",
    "yaw_tolerance_deg",
    "velocity_tolerance",
    "hold_time",
    "fly_through",
    "fly_through_tolerance",
    "timeout",
}

DEFAULTS = {
    "takeoff_height": None,
    "takeoff_settle_time": 0.0,
    "land_after_mission": True,
    "fly_through": True,
    "fly_through_tolerance": 0.5,
    "position_tolerance": 0.2,
    "yaw_tolerance_deg": 5.0,
    "velocity_tolerance": 0.15,
    "hold_time": 1.0,
    # Keep mission timing independent of the short-range local planning
    # horizon: obstacle avoidance and settling can make even a valid leg take
    # much longer than its straight-line distance suggests.
    "waypoint_timeout": 1200.0,
    "planner_accept_timeout": 15.0,
    # A collision check can publish a short emergency-stop trajectory and
    # recover with a new normal trajectory. Do not turn that recoverable
    # safety manoeuvre into an immediate mission-wide AUTO.LOITER request.
    "planner_recovery_timeout": 2.0,
    # Once an unrecoverable emergency stop has settled, Diff-Planner returns to
    # WAIT_TARGET. Re-submit the active (possibly substituted) waypoint with a
    # fresh stamp a bounded number of times.
    "planner_retry_limit": 3,
    # MAVROS State is commonly published near 1 Hz. Allow normal scheduling
    # jitter while still detecting a genuinely lost flight-state stream.
    "state_timeout": 3.0,
    "odom_timeout": 0.5,
}

# A rolling local map cannot authoritatively validate distant waypoints before
# the vehicle has moved near them.  These backend reason codes are therefore
# deferred during whole-mission preflight only; every waypoint is validated
# again immediately before it is published.
DEFERRED_PREFLIGHT_GOAL_REASONS = frozenset(
    {
        "goal_out_of_local_map",
    }
)


class MissionConfigError(ValueError):
    pass


def _finite_number(value, name, *, positive=False, nonnegative=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise MissionConfigError("{} must be a number".format(name))
    result = float(value)
    if not math.isfinite(result):
        raise MissionConfigError("{} must be finite".format(name))
    if positive and result <= 0.0:
        raise MissionConfigError("{} must be greater than zero".format(name))
    if nonnegative and result < 0.0:
        raise MissionConfigError("{} must not be negative".format(name))
    return result


def _validate_tolerance(value, name):
    return _finite_number(value, name, positive=True)


def _validate_nonnegative_integer(value, name):
    number = _finite_number(value, name, nonnegative=True)
    if not number.is_integer():
        raise MissionConfigError("{} must be an integer".format(name))
    return int(number)


def _segment_yaw_degrees(start, end, min_horizontal_distance):
    """Return a heading only when the horizontal baseline is meaningful."""
    dx = end["x"] - start["x"]
    dy = end["y"] - start["y"]
    if math.hypot(dx, dy) < min_horizontal_distance:
        return None
    return math.degrees(math.atan2(dy, dx))


def assign_automatic_waypoint_yaws(waypoints, min_horizontal_distance=0.5):
    """Fill missing yaw values from meaningful following route segments.

    Near-coincident x/y points are skipped because their direction is
    dominated by coordinate precision and does not describe a useful flight
    leg. Trailing points inherit the most recent valid automatic heading. If
    the whole route is horizontally shorter than the threshold, yaw remains
    unresolved until fresh odometry supplies the mission-start heading.
    An explicit yaw remains an opt-in override.
    """
    automatic_yaws = [None] * len(waypoints)
    for index, waypoint in enumerate(waypoints):
        for following in waypoints[index + 1 :]:
            yaw = _segment_yaw_degrees(
                waypoint, following, min_horizontal_distance
            )
            if yaw is not None:
                automatic_yaws[index] = yaw
                break

    # Points at the end of the route have no meaningful following segment.
    # Keep the last valid flight-leg direction instead of deriving a noisy yaw
    # from a near-coincident final pair.
    previous_yaw = None
    for index, yaw in enumerate(automatic_yaws):
        if yaw is not None:
            previous_yaw = yaw
        elif previous_yaw is not None:
            automatic_yaws[index] = previous_yaw

    # A rare cluster can have no direction from its first point while a later
    # pair in that cluster does. Backfill that first meaningful route heading.
    following_yaw = None
    for index in range(len(automatic_yaws) - 1, -1, -1):
        yaw = automatic_yaws[index]
        if yaw is not None:
            following_yaw = yaw
        elif following_yaw is not None:
            automatic_yaws[index] = following_yaw

    for waypoint, yaw in zip(waypoints, automatic_yaws):
        if waypoint["yaw"] is None and yaw is not None:
            waypoint["yaw"] = yaw


def fill_unresolved_waypoint_yaws(waypoints, current_yaw_degrees):
    """Use one current heading when a route has no horizontal direction."""
    count = 0
    for waypoint in waypoints:
        if waypoint["yaw"] is None:
            waypoint["yaw"] = current_yaw_degrees
            count += 1
    return count


def load_mission_config(
    path,
    *,
    default_takeoff_height=None,
    virtual_ground=None,
    virtual_ceil=None,
    obstacles_inflation=None
):
    if not os.path.isfile(path):
        raise MissionConfigError("mission file does not exist: {}".format(path))

    try:
        with open(path, "r", encoding="utf-8") as stream:
            raw = json.load(stream)
    except (OSError, json.JSONDecodeError) as exc:
        raise MissionConfigError("cannot read mission JSON: {}".format(exc))

    if not isinstance(raw, dict):
        raise MissionConfigError("mission root must be a JSON object")
    unknown = sorted(set(raw) - TOP_LEVEL_KEYS)
    if unknown:
        raise MissionConfigError(
            "unknown mission field(s): {}".format(", ".join(unknown))
        )

    config = dict(DEFAULTS)
    config.update(raw)

    takeoff_height = config["takeoff_height"]
    if takeoff_height is None:
        takeoff_height = default_takeoff_height
    if takeoff_height is None:
        raise MissionConfigError(
            "takeoff_height is required when no command default is supplied"
        )
    config["takeoff_height"] = _finite_number(
        takeoff_height, "takeoff_height", positive=True
    )
    safe_ground = (
        _finite_number(virtual_ground, "virtual_ground")
        if virtual_ground is not None
        else None
    )
    safe_ceil = (
        _finite_number(virtual_ceil, "virtual_ceil")
        if virtual_ceil is not None
        else None
    )
    if obstacles_inflation is not None:
        obstacles_inflation = _finite_number(
            obstacles_inflation, "obstacles_inflation", nonnegative=True
        )
        if safe_ground is not None:
            safe_ground += obstacles_inflation
        if safe_ceil is not None:
            safe_ceil -= obstacles_inflation
    if (
        safe_ground is not None
        and safe_ceil is not None
        and safe_ground >= safe_ceil
    ):
        raise MissionConfigError(
            "Planner vertical fence has no usable clearance after obstacle "
            "inflation"
        )
    if safe_ground is not None and config["takeoff_height"] <= safe_ground:
        raise MissionConfigError(
            "takeoff_height={} must be above the Planner ground clearance "
            "limit {}".format(config["takeoff_height"], safe_ground)
        )
    if safe_ceil is not None and config["takeoff_height"] >= safe_ceil:
        raise MissionConfigError(
            "takeoff_height={} must be below the Planner ceiling clearance "
            "limit {}".format(config["takeoff_height"], safe_ceil)
        )
    config["takeoff_settle_time"] = _finite_number(
        config["takeoff_settle_time"], "takeoff_settle_time", nonnegative=True
    )
    if not isinstance(config["land_after_mission"], bool):
        raise MissionConfigError("land_after_mission must be true or false")
    if not isinstance(config["fly_through"], bool):
        raise MissionConfigError("fly_through must be true or false")

    for key in (
        "position_tolerance",
        "yaw_tolerance_deg",
        "velocity_tolerance",
        "hold_time",
        "waypoint_timeout",
        "planner_accept_timeout",
        "planner_recovery_timeout",
        "state_timeout",
        "odom_timeout",
        "fly_through_tolerance",
    ):
        config[key] = _validate_tolerance(config[key], key)
    config["planner_retry_limit"] = _validate_nonnegative_integer(
        config["planner_retry_limit"], "planner_retry_limit"
    )
    if config["yaw_tolerance_deg"] > 180.0:
        raise MissionConfigError("yaw_tolerance_deg must not exceed 180")

    waypoints = config.get("waypoints")
    if not isinstance(waypoints, list) or not waypoints:
        raise MissionConfigError("waypoints must be a non-empty JSON array")

    normalized_waypoints = []
    for index, waypoint in enumerate(waypoints, start=1):
        prefix = "waypoints[{}]".format(index - 1)
        if not isinstance(waypoint, dict):
            raise MissionConfigError("{} must be a JSON object".format(prefix))
        unknown = sorted(set(waypoint) - WAYPOINT_KEYS)
        if unknown:
            raise MissionConfigError(
                "{} has unknown field(s): {}".format(prefix, ", ".join(unknown))
            )
        missing = [axis for axis in ("x", "y", "z") if axis not in waypoint]
        if missing:
            raise MissionConfigError(
                "{} is missing field(s): {}".format(prefix, ", ".join(missing))
            )

        normalized = {
            "x": _finite_number(waypoint["x"], "{}.x".format(prefix)),
            "y": _finite_number(waypoint["y"], "{}.y".format(prefix)),
            "z": _finite_number(waypoint["z"], "{}.z".format(prefix)),
            "yaw": None,
            "position_tolerance": _validate_tolerance(
                waypoint.get(
                    "position_tolerance", config["position_tolerance"]
                ),
                "{}.position_tolerance".format(prefix),
            ),
            "yaw_tolerance_deg": _validate_tolerance(
                waypoint.get(
                    "yaw_tolerance_deg", config["yaw_tolerance_deg"]
                ),
                "{}.yaw_tolerance_deg".format(prefix),
            ),
            "velocity_tolerance": _validate_tolerance(
                waypoint.get(
                    "velocity_tolerance", config["velocity_tolerance"]
                ),
                "{}.velocity_tolerance".format(prefix),
            ),
            "hold_time": _validate_tolerance(
                waypoint.get("hold_time", config["hold_time"]),
                "{}.hold_time".format(prefix),
            ),
            "fly_through": waypoint.get(
                "fly_through", config["fly_through"]
            ),
            "fly_through_tolerance": _validate_tolerance(
                waypoint.get(
                    "fly_through_tolerance",
                    config["fly_through_tolerance"],
                ),
                "{}.fly_through_tolerance".format(prefix),
            ),
            "timeout": _validate_tolerance(
                waypoint.get("timeout", config["waypoint_timeout"]),
                "{}.timeout".format(prefix),
            ),
        }
        if not isinstance(normalized["fly_through"], bool):
            raise MissionConfigError(
                "{}.fly_through must be true or false".format(prefix)
            )
        if normalized["yaw_tolerance_deg"] > 180.0:
            raise MissionConfigError(
                "{}.yaw_tolerance_deg must not exceed 180".format(prefix)
            )
        if "yaw" in waypoint and waypoint["yaw"] is not None:
            normalized["yaw"] = _finite_number(
                waypoint["yaw"], "{}.yaw".format(prefix)
            )

        if safe_ground is not None and normalized["z"] <= safe_ground:
            raise MissionConfigError(
                "{}.z={} must be above the Planner ground clearance limit "
                "{}".format(
                    prefix, normalized["z"], safe_ground
                )
            )
        if safe_ceil is not None and normalized["z"] >= safe_ceil:
            raise MissionConfigError(
                "{}.z={} must be below the Planner ceiling clearance limit "
                "{}".format(
                    prefix, normalized["z"], safe_ceil
                )
            )
        normalized_waypoints.append(normalized)

    assign_automatic_waypoint_yaws(
        normalized_waypoints, config["fly_through_tolerance"]
    )
    config["waypoints"] = normalized_waypoints
    return config


def _wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw_from_quaternion(quaternion):
    values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    if not all(math.isfinite(value) for value in values):
        return None
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-9:
        return None
    x, y, z, w = (value / norm for value in values)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def waypoint_is_fly_through(waypoint, index, count):
    """Intermediate opt-in points pass through; the final point always stops."""
    return bool(waypoint["fly_through"] and index < count)


def fly_through_arrival_ready(position_error, distance_tolerance, yaw_ok):
    """A fly-through point switches only after position and yaw are ready."""
    return position_error <= distance_tolerance and yaw_ok


def mission_failure_recovery_mode(reason):
    """Land when localization is lost; otherwise hold for planner failures."""
    if reason == LOCALIZATION_STALE_REASON:
        return "AUTO.LAND"
    return "AUTO.LOITER"


def localization_fault_reason(value):
    """Return the persistent interlock reason, or empty when it is clear."""
    if isinstance(value, dict):
        if not value.get("active", False):
            return ""
        return str(value.get("reason") or "localization safety fault")
    if value:
        return str(value)
    return ""


class WaypointMission:
    def __init__(
        self,
        config,
        drone_id,
        odometry_topic="/localization/odom",
        state_topic="/mavros/state",
        planner_status_topic="/planning/status",
        validate_goal_service="/planning/validate_goal",
    ):
        # ROS imports stay inside runtime construction so configuration
        # validation also works on a host that does not source ROS.
        import rospy
        from geometry_msgs.msg import PoseStamped
        from mavros_msgs.msg import State
        from mavros_msgs.srv import SetMode
        from nav_msgs.msg import Odometry
        from sim2real_planning_msgs.msg import PlannerGoal, PlannerStatus
        from sim2real_planning_msgs.srv import ValidateGoal
        from std_srvs.srv import Trigger

        self.rospy = rospy
        self.PoseStamped = PoseStamped
        self.SetMode = SetMode
        self.Trigger = Trigger
        self.PlannerGoal = PlannerGoal
        self.PlannerStatus = PlannerStatus
        self.config = config
        self.drone_id = drone_id
        self.state_topic = state_topic
        self.odometry_topic = odometry_topic
        self.lock = threading.Lock()

        self.state = None
        self.state_received_at = 0.0
        self.odom = None
        self.odom_received_at = 0.0
        self.current_goal_stamp = None
        self.current_goal_id = None
        self.current_goal_floor = 0
        self.last_planner_goal_id = 0
        self.planner_status = None
        self.planner_status_received_at = 0.0
        self.current_requested_goal_position = None
        self.current_goal_position = None
        self.current_goal_message = None
        self.current_plan_accepted = False
        self.localization_fault_latched = False
        self.current_planner_stopped_at = None
        self.current_planner_retry_count = 0
        self.abort_requested = False
        self.goals_validated = False

        self.goal_pub = rospy.Publisher(
            "/goal", PoseStamped, queue_size=1, latch=False
        )
        rospy.Subscriber(
            self.state_topic, State, self._state_callback, queue_size=1
        )
        rospy.Subscriber(
            self.odometry_topic,
            Odometry,
            self._odom_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            planner_status_topic,
            PlannerStatus,
            self._planner_status_callback,
            queue_size=10,
            tcp_nodelay=True,
        )
        self.set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.validate_goal = rospy.ServiceProxy(
            validate_goal_service, ValidateGoal
        )
        self.validate_goal_service = validate_goal_service
        self.cancel_goal = rospy.ServiceProxy("/planning/cancel", Trigger)

    def _state_callback(self, message):
        with self.lock:
            self.state = message
            self.state_received_at = time.monotonic()

    def _odom_callback(self, message):
        with self.lock:
            self.odom = message
            self.odom_received_at = time.monotonic()

    def _planner_status_callback(self, message):
        """Consume only the selected backend's public status contract."""
        if not hasattr(message, "goal_id"):
            # Kept as a compatibility seam for unit tests of the former
            # PolyTraj correlation logic; no native topic subscribes here.
            return self._legacy_trajectory_callback(message)

        fallback_update = None
        now = time.monotonic()
        with self.lock:
            self.planner_status = message
            self.planner_status_received_at = now
            goal_id = int(message.goal_id)
            self.last_planner_goal_id = max(self.last_planner_goal_id, goal_id)
            if self.current_goal_stamp is None or goal_id <= 0:
                return
            if self.current_goal_id is None:
                if goal_id <= self.current_goal_floor:
                    return
                self.current_goal_id = goal_id
            elif goal_id != self.current_goal_id:
                return

            state = int(message.state)
            goal_update_states = {
                int(getattr(message, "PLANNING", 2)),
                int(getattr(message, "ACTIVE", 3)),
                int(getattr(message, "REACHED", 5)),
            }
            active_goal = getattr(message, "active_goal", None)
            if active_goal is not None and state in goal_update_states:
                position = active_goal.pose.position
                goal = (float(position.x), float(position.y), float(position.z))
                if all(math.isfinite(value) for value in goal):
                    goal_difference = math.sqrt(
                        sum(
                            (goal[index] - self.current_goal_position[index]) ** 2
                            for index in range(3)
                        )
                    )
                    if goal_difference > 0.05:
                        fallback_update = (
                            self.current_requested_goal_position,
                            self.current_goal_position,
                            goal,
                        )
                        self.current_goal_position = goal

            active_states = {
                int(getattr(message, "ACTIVE", 3)),
                int(getattr(message, "REACHED", 5)),
            }
            stop_states = {
                int(getattr(message, "HOLDING", 4)),
                int(getattr(message, "FAULT", 6)),
            }
            if (
                state in active_states
                and bool(message.armable)
                and int(message.trajectory_id) > 0
            ):
                self.current_plan_accepted = True
                self.current_planner_stopped_at = None
            elif state in stop_states:
                if self.current_planner_stopped_at is None:
                    self.current_planner_stopped_at = now
        if fallback_update is not None:
            self._log_fallback(fallback_update)

    def _trajectory_callback(self, message):
        """Backward-compatible test entrypoint; runtime uses PlannerStatus."""
        return self._planner_status_callback(message)

    def _legacy_trajectory_callback(self, message):
        fallback_update = None
        with self.lock:
            if self.current_goal_stamp is None:
                return
            if message.goal_stamp != self.current_goal_stamp:
                return
            goal = tuple(float(value) for value in message.goal_position)
            if not all(math.isfinite(value) for value in goal):
                return
            goal_difference = math.sqrt(
                sum(
                    (goal[index] - self.current_goal_position[index]) ** 2
                    for index in range(3)
                )
            )
            if message.armable and message.traj_id > 0:
                if goal_difference > 0.05:
                    fallback_update = (
                        self.current_requested_goal_position,
                        self.current_goal_position,
                        goal,
                    )
                    self.current_goal_position = goal
                self.current_plan_accepted = True
                self.current_planner_stopped_at = None
            elif message.traj_id > 0:
                # An initial plan can fail and publish an emergency-stop
                # trajectory before any armable trajectory is accepted.  That
                # is still a planner stop for this stamped goal and must enter
                # the bounded retry path instead of waiting out the full
                # acceptance timeout.
                if self.current_planner_stopped_at is None:
                    self.current_planner_stopped_at = time.monotonic()
        if fallback_update is not None:
            self._log_fallback(fallback_update)

    def _log_fallback(self, fallback_update):
        requested, previous, fallback = fallback_update
        self.rospy.logwarn(
                "Planner replaced mission waypoint "
                "(%.3f, %.3f, %.3f) with a collision-free fallback "
                "(%.3f, %.3f, %.3f); accepting the fallback and continuing. "
                "Previous active target was (%.3f, %.3f, %.3f).",
                requested[0],
                requested[1],
                requested[2],
                fallback[0],
                fallback[1],
                fallback[2],
                previous[0],
                previous[1],
            previous[2],
        )

    def _validate_waypoint(
        self,
        waypoint,
        index,
        count,
        session_id,
        *,
        defer_dynamic_map_rejections=False,
    ):
        pose = self.PoseStamped()
        pose.header.stamp = self.rospy.Time.now()
        pose.header.frame_id = "world"
        pose.pose.position.x = waypoint["x"]
        pose.pose.position.y = waypoint["y"]
        pose.pose.position.z = waypoint["z"]
        constrain_yaw = waypoint["yaw"] is not None
        if constrain_yaw:
            yaw = math.radians(waypoint["yaw"])
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)

        goal = self.PlannerGoal()
        goal.header.stamp = pose.header.stamp
        goal.session_id = session_id
        goal.goal_id = index
        goal.action = goal.PLAN
        goal.goal = pose
        goal.constrain_yaw = constrain_yaw
        try:
            response = self.validate_goal(goal)
        except Exception as exc:
            return (
                False,
                "selected planner goal validation failed for waypoint "
                "{}/{}: {}".format(index, count, exc),
            )
        if not response.valid:
            backend_reason = (response.reason or "").strip()
            if (
                defer_dynamic_map_rejections
                and backend_reason in DEFERRED_PREFLIGHT_GOAL_REASONS
            ):
                self.rospy.loginfo(
                    "Deferring selected planner validation for waypoint "
                    "%d/%d until dispatch: %s",
                    index,
                    count,
                    backend_reason,
                )
                return True, ""
            return (
                False,
                "selected planner rejected waypoint {}/{}: {}".format(
                    index,
                    count,
                    backend_reason or "unspecified validation failure",
                ),
            )
        return True, ""

    def validate_all_goals(self):
        """Preflight goals, deferring only rolling-map range checks."""
        if self.goals_validated:
            return True, ""
        try:
            self.rospy.wait_for_service(
                self.validate_goal_service,
                timeout=self.config["planner_accept_timeout"],
            )
        except Exception as exc:
            return False, "selected planner goal validation failed: {}".format(exc)

        count = len(self.config["waypoints"])
        for index, waypoint in enumerate(self.config["waypoints"], start=1):
            valid, reason = self._validate_waypoint(
                waypoint,
                index,
                count,
                "mission-preflight",
                defer_dynamic_map_rejections=True,
            )
            if not valid:
                return False, reason
        self.goals_validated = True
        return True, ""

    def _snapshot(self):
        with self.lock:
            return (
                self.state,
                self.state_received_at,
                self.odom,
                self.odom_received_at,
                self.current_plan_accepted,
                self.current_planner_stopped_at,
                self.current_goal_position,
            )

    def _planner_stop_is_persistent(self, stopped_at):
        if stopped_at is None:
            return False
        elapsed = time.monotonic() - stopped_at
        timeout = self.config["planner_recovery_timeout"]
        if elapsed >= timeout:
            return True
        self.rospy.logwarn_throttle(
            1.0,
            "Planner issued a temporary emergency-stop trajectory; "
            "waiting up to %.1f s for automatic replanning.",
            timeout,
        )
        return False

    def _retry_active_goal(self):
        with self.lock:
            retry_limit = self.config["planner_retry_limit"]
            if self.current_planner_retry_count >= retry_limit:
                return (
                    False,
                    "Planner exhausted {} retries for the current waypoint".format(
                        retry_limit
                    ),
                )
            if self.current_goal_message is None or self.current_goal_position is None:
                return False, "Planner stopped without an active waypoint to retry"

            self.current_planner_retry_count += 1
            retry_count = self.current_planner_retry_count
            active_goal = self.current_goal_position
            message = self.current_goal_message
            message.header.stamp = self.rospy.Time.now()
            message.pose.position.x = active_goal[0]
            message.pose.position.y = active_goal[1]
            message.pose.position.z = active_goal[2]
            self.current_goal_stamp = message.header.stamp
            self.current_goal_floor = getattr(self, "last_planner_goal_id", 0)
            self.current_goal_id = None
            self.current_plan_accepted = False
            self.current_planner_stopped_at = None

        self.rospy.logwarn(
            "Planner did not automatically recover from its emergency stop. "
            "Re-submitting active waypoint (%.3f, %.3f, %.3f) with a fresh "
            "goal stamp (retry %d/%d).",
            active_goal[0],
            active_goal[1],
            active_goal[2],
            retry_count,
            retry_limit,
        )
        self.goal_pub.publish(message)
        return True, ""

    def _cancel_completed_goal(self):
        try:
            self.rospy.wait_for_service(
                "/planning/cancel", timeout=self.config["state_timeout"]
            )
            response = self.cancel_goal()
        except Exception as exc:
            return False, "planner cancellation failed: {}".format(exc)
        if not response.success:
            return False, response.message or "planner rejected cancellation"
        with self.lock:
            self.current_plan_accepted = False
            self.current_planner_stopped_at = None
            self.current_goal_stamp = None
            self.current_goal_id = None
        return True, ""

    def request_abort(self):
        with self.lock:
            self.abort_requested = True

    def _wall_wait(self, duration=0.05):
        """Wait using wall time so a paused Gazebo /clock cannot deadlock."""
        time.sleep(duration)

    def _latch_localization_fault(self):
        with self.lock:
            if self.localization_fault_latched:
                return
            self.localization_fault_latched = True
        try:
            self.rospy.set_param(
                LOCALIZATION_FAULT_PARAM,
                {"active": True, "reason": LOCALIZATION_STALE_REASON},
            )
        except Exception as exc:
            self.rospy.logerr(
                "Could not persist the localization safety interlock: %s", exc
            )
        self.rospy.logerr(
            "Localization safety interlock latched. Restart the complete "
            "simulation/real stack before another autonomous command."
        )

    def _flight_gate(self):
        with self.lock:
            abort_requested = self.abort_requested
        if abort_requested:
            return EXIT_MISSION_FAILED, "mission runner was interrupted"
        try:
            fault_reason = localization_fault_reason(
                self.rospy.get_param(LOCALIZATION_FAULT_PARAM, "")
            )
        except Exception as exc:
            return (
                EXIT_MISSION_FAILED,
                "cannot read localization safety interlock: {}".format(exc),
            )
        if fault_reason:
            with self.lock:
                self.localization_fault_latched = True
            return EXIT_MISSION_FAILED, LOCALIZATION_STALE_REASON
        state, state_time, odom, odom_time, _, _, _ = self._snapshot()
        now = time.monotonic()
        if state is None or now - state_time > self.config["state_timeout"]:
            return EXIT_MISSION_FAILED, "MAVROS state is unavailable or stale"
        if not state.connected:
            return EXIT_MISSION_FAILED, "PX4 disconnected"
        if not state.armed:
            return EXIT_MANUAL_TAKEOVER, "vehicle is no longer armed"
        if state.mode != "OFFBOARD":
            return (
                EXIT_MANUAL_TAKEOVER,
                "flight mode changed from OFFBOARD to {}".format(state.mode),
            )
        if odom is None or now - odom_time > self.config["odom_timeout"]:
            self._latch_localization_fault()
            return EXIT_MISSION_FAILED, LOCALIZATION_STALE_REASON
        with self.lock:
            planner_status = self.planner_status
            planner_status_at = self.planner_status_received_at
        if (
            planner_status is None
            or now - planner_status_at > self.config["state_timeout"]
        ):
            return EXIT_MISSION_FAILED, "selected planner status is unavailable or stale"
        if (
            int(planner_status.state)
            == int(getattr(planner_status, "FAULT", 6))
        ):
            return (
                EXIT_MISSION_FAILED,
                "selected planner fault: {}".format(
                    planner_status.reason or "unspecified fault"
                ),
            )
        if not planner_status.odom_ready or not planner_status.map_ready:
            return EXIT_MISSION_FAILED, "selected planner is not map/odom ready"
        return EXIT_SUCCESS, ""

    def _request_recovery(self, reason):
        state, state_time, _, _, _, _, _ = self._snapshot()
        if state is None or not state.connected or not state.armed:
            return
        if time.monotonic() - state_time > self.config["state_timeout"]:
            self.rospy.logerr(
                "Cannot confirm a recovery mode because MAVROS state is stale; "
                "take over immediately with the RC."
            )
            return
        if state.mode not in ("OFFBOARD", "AUTO.TAKEOFF", "AUTO.LOITER"):
            self.rospy.logwarn(
                "Mission stopped after mode changed to %s; not overriding the pilot.",
                state.mode,
            )
            return
        recovery_mode = mission_failure_recovery_mode(reason)
        if recovery_mode == "AUTO.LAND":
            self.rospy.logerr(
                "Mission failure: %s. Localization cannot support a safe "
                "position hold; requesting and confirming PX4 AUTO.LAND.",
                reason,
            )
        else:
            self.rospy.logerr(
                "Mission failure: %s. Requesting and confirming PX4 "
                "AUTO.LOITER; RC takeover remains available.",
                reason,
            )

        deadline = time.monotonic() + self.config["state_timeout"]
        while not self.rospy.is_shutdown() and time.monotonic() < deadline:
            state, state_time, _, _, _, _, _ = self._snapshot()
            if state is None or not state.connected or not state.armed:
                return
            if time.monotonic() - state_time > self.config["state_timeout"]:
                break
            if state.mode == recovery_mode:
                self.rospy.logwarn(
                    "PX4 %s is active after mission failure.", recovery_mode
                )
                return
            if state.mode not in ("OFFBOARD", "AUTO.TAKEOFF", "AUTO.LOITER"):
                self.rospy.logwarn(
                    "Mission recovery stopped after mode changed to %s; not "
                    "overriding the pilot.",
                    state.mode or "unknown",
                )
                return
            try:
                response = self.set_mode(
                    base_mode=0, custom_mode=recovery_mode
                )
                if not response.mode_sent:
                    self.rospy.logerr_throttle(
                        1.0, "PX4 rejected %s during mission recovery.", recovery_mode
                    )
            except Exception as exc:  # service exceptions vary by ROS release
                self.rospy.logerr_throttle(
                    1.0,
                    "Unable to request %s (%s).",
                    recovery_mode,
                    exc,
                )
            self._wall_wait()
        self.rospy.logerr(
            "PX4 did not enter %s within %.1f s; take over "
            "immediately with the RC.",
            recovery_mode,
            self.config["state_timeout"],
        )

    def _wait_for_initial_state(self):
        deadline = time.monotonic() + self.config["planner_accept_timeout"]
        while not self.rospy.is_shutdown() and time.monotonic() < deadline:
            code, reason = self._flight_gate()
            if code == EXIT_SUCCESS and self.goal_pub.get_num_connections() >= 1:
                return EXIT_SUCCESS, ""
            if code == EXIT_MANUAL_TAKEOVER:
                return code, reason
            if (
                code == EXIT_MISSION_FAILED
                and reason == LOCALIZATION_STALE_REASON
            ):
                # The localization interlock is irreversible for this stack
                # lifetime. Preserve that reason so recovery requests
                # AUTO.LAND instead of misclassifying it as a topic timeout.
                return code, reason
            self._wall_wait()
        return EXIT_MISSION_FAILED, "mission topics or fresh flight state are not ready"

    def _resolve_runtime_yaws(self):
        unresolved = sum(
            waypoint["yaw"] is None for waypoint in self.config["waypoints"]
        )
        if unresolved == 0:
            return True, ""

        _, _, odom, _, _, _, _ = self._snapshot()
        if odom is None:
            return False, "cannot lock mission yaw without localization odometry"
        current_yaw = _yaw_from_quaternion(odom.pose.pose.orientation)
        if current_yaw is None:
            return False, "cannot lock mission yaw from an invalid orientation"
        current_yaw_degrees = math.degrees(current_yaw)
        filled = fill_unresolved_waypoint_yaws(
            self.config["waypoints"], current_yaw_degrees
        )
        self.rospy.loginfo(
            "No mission waypoint pair is separated by the %.2f m automatic "
            "yaw baseline; locked %d waypoint yaw value(s) to the current "
            "heading %.1f deg.",
            self.config["fly_through_tolerance"],
            filled,
            current_yaw_degrees,
        )
        return True, ""

    def _publish_waypoint(self, waypoint, index, count):
        message = self.PoseStamped()
        message.header.stamp = self.rospy.Time.now()
        message.header.frame_id = "world"
        message.pose.position.x = waypoint["x"]
        message.pose.position.y = waypoint["y"]
        message.pose.position.z = waypoint["z"]
        if waypoint["yaw"] is None:
            # The Planner contract uses a zero-norm quaternion for an
            # unconstrained/path-aligned final yaw.
            message.pose.orientation.x = 0.0
            message.pose.orientation.y = 0.0
            message.pose.orientation.z = 0.0
            message.pose.orientation.w = 0.0
            yaw_text = "unconstrained"
        else:
            yaw = math.radians(waypoint["yaw"])
            message.pose.orientation.z = math.sin(yaw / 2.0)
            message.pose.orientation.w = math.cos(yaw / 2.0)
            yaw_text = "{:.1f} deg".format(waypoint["yaw"])

        with self.lock:
            self.current_goal_stamp = message.header.stamp
            self.current_goal_floor = self.last_planner_goal_id
            self.current_goal_id = None
            requested_position = (
                waypoint["x"],
                waypoint["y"],
                waypoint["z"],
            )
            self.current_requested_goal_position = requested_position
            self.current_goal_position = requested_position
            self.current_goal_message = message
            self.current_plan_accepted = False
            self.current_planner_stopped_at = None
            self.current_planner_retry_count = 0

        self.rospy.loginfo(
            "Mission waypoint %d/%d: position=(%.3f, %.3f, %.3f), yaw=%s, "
            "goal subscribers=%d",
            index,
            count,
            waypoint["x"],
            waypoint["y"],
            waypoint["z"],
            yaw_text,
            self.goal_pub.get_num_connections(),
        )
        self.goal_pub.publish(message)

    def _wait_for_plan(self):
        deadline = time.monotonic() + self.config["planner_accept_timeout"]
        while not self.rospy.is_shutdown() and time.monotonic() < deadline:
            code, reason = self._flight_gate()
            if code != EXIT_SUCCESS:
                return code, reason
            _, _, _, _, accepted, stopped_at, _ = self._snapshot()
            if self._planner_stop_is_persistent(stopped_at):
                retried, retry_reason = self._retry_active_goal()
                if not retried:
                    return EXIT_MISSION_FAILED, retry_reason
                deadline = (
                    time.monotonic() + self.config["planner_accept_timeout"]
                )
                self._wall_wait()
                continue
            if stopped_at is not None:
                self._wall_wait()
                continue
            if accepted:
                return EXIT_SUCCESS, ""
            self._wall_wait()
        return EXIT_MISSION_FAILED, "Planner did not accept the waypoint in time"

    def _wait_for_arrival(self, waypoint, *, fly_through=False):
        deadline = time.monotonic() + waypoint["timeout"]
        stable_since = None
        last_log = 0.0
        planner_accept_deadline = None
        while not self.rospy.is_shutdown() and time.monotonic() < deadline:
            code, reason = self._flight_gate()
            if code != EXIT_SUCCESS:
                return code, reason
            _, _, odom, _, accepted, stopped_at, active_goal = self._snapshot()
            if self._planner_stop_is_persistent(stopped_at):
                retried, retry_reason = self._retry_active_goal()
                if not retried:
                    return EXIT_MISSION_FAILED, retry_reason
                stable_since = None
                planner_accept_deadline = (
                    time.monotonic() + self.config["planner_accept_timeout"]
                )
                self._wall_wait()
                continue
            if stopped_at is not None:
                stable_since = None
                self._wall_wait()
                continue
            if not accepted:
                stable_since = None
                if (
                    planner_accept_deadline is not None
                    and time.monotonic() >= planner_accept_deadline
                ):
                    return (
                        EXIT_MISSION_FAILED,
                        "Planner did not accept the retried waypoint in time",
                    )
                self._wall_wait()
                continue
            planner_accept_deadline = None

            position = odom.pose.pose.position
            dx = position.x - active_goal[0]
            dy = position.y - active_goal[1]
            dz = position.z - active_goal[2]
            position_error = math.sqrt(dx * dx + dy * dy + dz * dz)
            velocity = odom.twist.twist.linear
            speed = math.sqrt(
                velocity.x * velocity.x
                + velocity.y * velocity.y
                + velocity.z * velocity.z
            )

            yaw_error_deg = 0.0
            yaw_ok = True
            if waypoint["yaw"] is not None:
                current_yaw = _yaw_from_quaternion(odom.pose.pose.orientation)
                if current_yaw is None:
                    yaw_ok = False
                    yaw_error_deg = float("inf")
                else:
                    yaw_error_deg = abs(
                        math.degrees(
                            _wrap_angle(
                                math.radians(waypoint["yaw"]) - current_yaw
                            )
                        )
                    )
                    yaw_ok = yaw_error_deg <= waypoint["yaw_tolerance_deg"]

            # Intermediate fly-through points are switching surfaces, not
            # stopping goals. Require the configured heading before sending
            # the next goal, but do not require low speed or a hold time.
            if (
                fly_through
                and fly_through_arrival_ready(
                    position_error,
                    waypoint["fly_through_tolerance"],
                    yaw_ok,
                )
            ):
                if waypoint["yaw"] is None:
                    self.rospy.loginfo(
                        "Fly-through radius reached: position error=%.2f m, "
                        "speed=%.2f m/s, yaw=unconstrained",
                        position_error,
                        speed,
                    )
                else:
                    self.rospy.loginfo(
                        "Fly-through position and yaw reached: position "
                        "error=%.2f m, yaw error=%.1f deg, speed=%.2f m/s",
                        position_error,
                        yaw_error_deg,
                        speed,
                    )
                return EXIT_SUCCESS, ""

            inside = (
                position_error <= waypoint["position_tolerance"]
                and speed <= waypoint["velocity_tolerance"]
                and yaw_ok
            )
            now = time.monotonic()
            if inside:
                if stable_since is None:
                    stable_since = now
                if now - stable_since >= waypoint["hold_time"]:
                    return EXIT_SUCCESS, ""
            else:
                stable_since = None

            if now - last_log >= 2.0:
                if waypoint["yaw"] is None:
                    self.rospy.loginfo(
                        "Waiting for waypoint: position error=%.2f m, speed=%.2f m/s",
                        position_error,
                        speed,
                    )
                else:
                    self.rospy.loginfo(
                        "Waiting for waypoint: position error=%.2f m, speed=%.2f m/s, yaw error=%.1f deg",
                        position_error,
                        speed,
                        yaw_error_deg,
                    )
                last_log = now
            self._wall_wait()

        return EXIT_MISSION_FAILED, "waypoint arrival timed out"

    def run(self):
        valid, reason = self.validate_all_goals()
        if not valid:
            self.rospy.logerr("Mission validation failed: %s", reason)
            return EXIT_MISSION_FAILED
        code, reason = self._wait_for_initial_state()
        if code != EXIT_SUCCESS:
            if code == EXIT_MISSION_FAILED:
                self._request_recovery(reason)
            return code

        yaw_ready, reason = self._resolve_runtime_yaws()
        if not yaw_ready:
            self._request_recovery(reason)
            return EXIT_MISSION_FAILED

        count = len(self.config["waypoints"])
        for index, waypoint in enumerate(self.config["waypoints"], start=1):
            fly_through = waypoint_is_fly_through(waypoint, index, count)
            valid, reason = self._validate_waypoint(
                waypoint, index, count, "mission-runtime"
            )
            if not valid:
                self._request_recovery(reason)
                return EXIT_MISSION_FAILED
            self._publish_waypoint(waypoint, index, count)
            code, reason = self._wait_for_plan()
            if code != EXIT_SUCCESS:
                if code == EXIT_MISSION_FAILED:
                    self._request_recovery(reason)
                else:
                    self.rospy.logwarn(
                        "Mission aborted for manual takeover: %s", reason
                    )
                return code

            code, reason = self._wait_for_arrival(
                waypoint, fly_through=fly_through
            )
            if code != EXIT_SUCCESS:
                if code == EXIT_MISSION_FAILED:
                    self._request_recovery(reason)
                else:
                    self.rospy.logwarn(
                        "Mission aborted for manual takeover: %s", reason
                    )
                return code
            _, _, _, _, _, _, active_goal = self._snapshot()
            requested_goal = (waypoint["x"], waypoint["y"], waypoint["z"])
            fallback_distance = math.sqrt(
                sum(
                    (active_goal[axis] - requested_goal[axis]) ** 2
                    for axis in range(3)
                )
            )
            if fly_through:
                self.rospy.loginfo(
                    "Mission waypoint %d/%d passed without stopping.",
                    index,
                    count,
                )
            elif fallback_distance > 0.05:
                self.rospy.loginfo(
                    "Mission waypoint %d/%d reached at Planner fallback "
                    "(%.3f, %.3f, %.3f).",
                    index,
                    count,
                    active_goal[0],
                    active_goal[1],
                    active_goal[2],
                )
            else:
                self.rospy.loginfo(
                    "Mission waypoint %d/%d reached.", index, count
                )

        cancelled, reason = self._cancel_completed_goal()
        if not cancelled:
            self._request_recovery(reason)
            return EXIT_MISSION_FAILED
        self.rospy.loginfo(
            "All %d mission waypoints reached; active planner goal cancelled.",
            count,
        )
        return EXIT_SUCCESS


def _build_parser():
    parser = argparse.ArgumentParser(
        description="Validate or execute an ordered waypoint mission"
    )
    parser.add_argument("mission_file")
    parser.add_argument("--drone-id", type=int, default=0)
    parser.add_argument("--state-topic", default="/mavros/state")
    parser.add_argument("--odometry-topic", default="/localization/odom")
    parser.add_argument("--validate-only", action="store_true")
    parser.add_argument("--shell-summary", action="store_true")
    parser.add_argument("--default-takeoff-height", type=float)
    parser.add_argument("--virtual-ground", type=float)
    parser.add_argument("--virtual-ceil", type=float)
    return parser


def main(argv=None):
    args = _build_parser().parse_args(argv)
    if (args.virtual_ground is None) != (args.virtual_ceil is None):
        print(
            "[ERROR] --virtual-ground and --virtual-ceil must be supplied together",
            file=sys.stderr,
        )
        return EXIT_MISSION_FAILED
    if (
        args.virtual_ground is not None
        and args.virtual_ground >= args.virtual_ceil
    ):
        print(
            "[ERROR] virtual_ground must be below virtual_ceil",
            file=sys.stderr,
        )
        return EXIT_MISSION_FAILED

    try:
        config = load_mission_config(
            args.mission_file,
            default_takeoff_height=args.default_takeoff_height,
            virtual_ground=args.virtual_ground,
            virtual_ceil=args.virtual_ceil,
        )
    except MissionConfigError as exc:
        print("[ERROR] Invalid waypoint mission: {}".format(exc), file=sys.stderr)
        return EXIT_MISSION_FAILED

    if args.shell_summary:
        print(
            "{}\t{}\t{}".format(
                config["takeoff_height"],
                config["takeoff_settle_time"],
                "true" if config["land_after_mission"] else "false",
            )
        )
        return EXIT_SUCCESS
    if args.validate_only:
        print(
            "[INFO] Mission is valid: {} ordered waypoint(s).".format(
                len(config["waypoints"])
            )
        )
        return EXIT_SUCCESS

    try:
        import rospy

        rospy.init_node("waypoint_mission", disable_signals=True)
        runner = WaypointMission(
            config,
            args.drone_id,
            state_topic=args.state_topic,
            odometry_topic=args.odometry_topic,
        )
        previous_sigint = signal.getsignal(signal.SIGINT)
        previous_sigterm = signal.getsignal(signal.SIGTERM)

        def request_abort(_signum, _frame):
            runner.request_abort()

        signal.signal(signal.SIGINT, request_abort)
        signal.signal(signal.SIGTERM, request_abort)
        try:
            try:
                result = runner.run()
            except Exception as exc:
                rospy.logerr("Waypoint mission crashed: %s", exc)
                runner._request_recovery(
                    "unexpected waypoint mission error: {}".format(exc)
                )
                result = EXIT_MISSION_FAILED
        finally:
            signal.signal(signal.SIGINT, previous_sigint)
            signal.signal(signal.SIGTERM, previous_sigterm)
            rospy.signal_shutdown("waypoint mission finished")
        return result
    except ImportError as exc:
        print(
            "[ERROR] ROS mission runtime dependency is unavailable: {}".format(
                exc
            ),
            file=sys.stderr,
        )
        return EXIT_MISSION_FAILED
    except KeyboardInterrupt:
        return EXIT_MISSION_FAILED
    except Exception as exc:
        try:
            import rospy

            rospy.logerr("Waypoint mission crashed: %s", exc)
        except ImportError:
            print("[ERROR] Waypoint mission crashed: {}".format(exc), file=sys.stderr)
        return EXIT_MISSION_FAILED


if __name__ == "__main__":
    sys.exit(main())
