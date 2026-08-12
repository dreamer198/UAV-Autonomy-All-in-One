#!/usr/bin/env python3
"""Guarded RViz target action for the fixed single-UAV real-flight stack.

The server deliberately delegates arming/takeoff and goal publication to the
repository's shared executors.  It adds the operator-facing action boundary,
fresh landed-state interlock, pre-arm planner validation, cancellation, and a
lock shared with ``launch/real.sh``.
"""

import fcntl
import math
import os
import subprocess
import sys
import threading
import time


ON_GROUND = 1
MIN_TAKEOFF_HEIGHT = 0.5
MAX_TAKEOFF_HEIGHT = 2.5
DEFAULT_CHILD_STOP_TIMEOUT = 3.0


class InteractiveGoalError(RuntimeError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = int(code)


def finite_positive(value):
    return math.isfinite(float(value)) and float(value) > 0.0


def child_stop_timeout(monitor_arm, command_timeout):
    """Allow an interrupted arm executor to finish LOITER/LAND recovery."""
    if not monitor_arm:
        return DEFAULT_CHILD_STOP_TIMEOUT
    # arm_executor can spend one command timeout requesting AUTO.LOITER and a
    # second one falling back to AUTO.LAND.  Killing it after the generic
    # three-second process timeout would cut that safety recovery short.
    return max(
        DEFAULT_CHILD_STOP_TIMEOUT,
        2.0 * float(command_timeout) + 2.0,
    )


def quaternion_yaw_degrees(orientation):
    values = (
        float(orientation.x),
        float(orientation.y),
        float(orientation.z),
        float(orientation.w),
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("target orientation must be finite")
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-6:
        return None
    x, y, z, w = (value / norm for value in values)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.degrees(yaw)


def validate_goal_request(goal):
    position = goal.target.pose.position
    coordinates = (float(position.x), float(position.y), float(position.z))
    if not all(math.isfinite(value) for value in coordinates):
        raise ValueError("target coordinates must be finite")
    frame_id = (goal.target.header.frame_id or "world").strip()
    if frame_id != "world":
        raise ValueError("target frame must be 'world'")
    if not finite_positive(goal.takeoff_height) or not (
        MIN_TAKEOFF_HEIGHT
        <= float(goal.takeoff_height)
        <= MAX_TAKEOFF_HEIGHT
    ):
        raise ValueError(
            "takeoff height must be finite and within [{:.1f}, {:.1f}] m".format(
                MIN_TAKEOFF_HEIGHT, MAX_TAKEOFF_HEIGHT
            )
        )
    yaw_degrees = quaternion_yaw_degrees(goal.target.pose.orientation)
    return coordinates, yaw_degrees


def vehicle_request_kind(
    *,
    connected,
    armed,
    mode,
    state_age,
    landed_state,
    extended_state_age,
    state_timeout,
    auto_arm_if_grounded,
):
    """Classify a request without importing ROS, so safety rules are testable."""
    if state_age > state_timeout:
        raise ValueError("MAVROS state is stale")
    if not connected:
        raise ValueError("MAVROS is not connected to PX4")
    if armed:
        if mode != "OFFBOARD":
            raise ValueError(
                "armed vehicle must already be in OFFBOARD (mode={})".format(
                    mode or "unknown"
                )
            )
        return "airborne_offboard"
    if not auto_arm_if_grounded:
        raise ValueError("ground auto-arm was not confirmed by the operator")
    if extended_state_age > state_timeout:
        raise ValueError("MAVROS extended state is stale")
    if int(landed_state) != ON_GROUND:
        raise ValueError("vehicle is not reliably ON_GROUND")
    return "disarmed_ground"


class InteractiveGoalServer:
    def __init__(self):
        import actionlib
        import rospy
        import rospkg
        from mavros_msgs.msg import ExtendedState, State
        from nav_msgs.msg import Odometry
        from sim2real_planning_msgs.msg import (
            InteractiveGoalAction,
            InteractiveGoalFeedback,
            InteractiveGoalResult,
            PlannerGoal,
            PlannerStatus,
        )
        from sim2real_planning_msgs.srv import ValidateGoal

        self.rospy = rospy
        self.InteractiveGoalFeedback = InteractiveGoalFeedback
        self.InteractiveGoalResult = InteractiveGoalResult
        self.PlannerGoal = PlannerGoal
        self.PlannerStatus = PlannerStatus
        self._condition = threading.Condition()
        self._state = None
        self._state_received = 0.0
        self._extended_state = None
        self._extended_state_received = 0.0
        self._odom_received = 0.0
        self._planner_status = None
        self._planner_status_received = 0.0
        self._child = None
        self._child_stop_timeout = DEFAULT_CHILD_STOP_TIMEOUT

        self.action_name = rospy.get_param(
            "~action_name", "/ground_station/interactive_goal"
        )
        self.lock_path = rospy.get_param(
            "~lock_path", "/root/tmp/real.lifecycle.lock"
        )
        self.state_timeout = float(rospy.get_param("~state_timeout", 3.0))
        self.odom_timeout = float(rospy.get_param("~odom_timeout", 0.5))
        self.planner_status_timeout = float(
            rospy.get_param("~planner_status_timeout", 0.5)
        )
        self.preflight_timeout = float(
            rospy.get_param("~preflight_timeout", 5.0)
        )
        self.command_timeout = float(rospy.get_param("~command_timeout", 15.0))
        self.takeoff_timeout = float(rospy.get_param("~takeoff_timeout", 30.0))
        self.takeoff_tolerance = float(
            rospy.get_param("~takeoff_tolerance", 0.1)
        )
        self.takeoff_stable_time = float(
            rospy.get_param("~takeoff_stable_time", 0.5)
        )
        self.takeoff_max_vertical_speed = float(
            rospy.get_param("~takeoff_max_vertical_speed", 0.2)
        )
        self.takeoff_altitude_field = rospy.get_param(
            "~takeoff_altitude_field", "relative"
        )
        self.disarmed_prearm_mode = rospy.get_param(
            "~disarmed_prearm_mode", "STABILIZED"
        )
        self.px4_hover_thrust = rospy.get_param("~px4_hover_thrust", None)
        if self.takeoff_altitude_field not in ("relative", "local", "auto"):
            raise ValueError("invalid takeoff_altitude_field")
        if self.disarmed_prearm_mode not in ("STABILIZED", "AUTO.LOITER"):
            raise ValueError("invalid disarmed_prearm_mode")
        if self.px4_hover_thrust is not None and not finite_positive(
            self.px4_hover_thrust
        ):
            raise ValueError("px4_hover_thrust must be finite and positive")
        self.controller_node = rospy.get_param(
            "~controller_node", "/se3_controller_node"
        )
        self.odometry_topic = rospy.get_param(
            "~odometry_topic", "/localization/odom"
        )
        self.attitude_setpoint_topic = rospy.get_param(
            "~attitude_setpoint_topic", "/mavros/setpoint_raw/attitude"
        )
        self.validate_goal_service_name = rospy.get_param(
            "~validate_goal_service", "/planning/validate_goal"
        )

        package_path = rospkg.RosPack().get_path("sim2real_common")
        self.arm_executor = os.path.join(
            package_path, "scripts", "arm_executor.py"
        )
        self.goal_executor = os.path.join(
            package_path, "scripts", "goal_executor.py"
        )
        for executable in (self.arm_executor, self.goal_executor):
            if not os.path.isfile(executable):
                raise RuntimeError("shared executor is missing: {}".format(executable))

        rospy.Subscriber(
            "/mavros/state", State, self._state_callback, queue_size=1
        )
        rospy.Subscriber(
            "/mavros/extended_state",
            ExtendedState,
            self._extended_state_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            self.odometry_topic,
            Odometry,
            self._odom_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            "/planning/status",
            PlannerStatus,
            self._planner_status_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        self.validate_goal = rospy.ServiceProxy(
            self.validate_goal_service_name, ValidateGoal
        )
        self.server = actionlib.SimpleActionServer(
            self.action_name,
            InteractiveGoalAction,
            execute_cb=self._execute,
            auto_start=False,
        )
        self.server.start()
        rospy.on_shutdown(self._stop_child)
        rospy.loginfo("Guarded interactive goal action ready: %s", self.action_name)

    def _state_callback(self, message):
        with self._condition:
            self._state = message
            self._state_received = time.monotonic()
            self._condition.notify_all()

    def _extended_state_callback(self, message):
        with self._condition:
            self._extended_state = message
            self._extended_state_received = time.monotonic()
            self._condition.notify_all()

    def _odom_callback(self, _message):
        with self._condition:
            self._odom_received = time.monotonic()
            self._condition.notify_all()

    def _planner_status_callback(self, message):
        with self._condition:
            self._planner_status = message
            self._planner_status_received = time.monotonic()
            self._condition.notify_all()

    def _feedback(self, stage, message):
        feedback = self.InteractiveGoalFeedback()
        feedback.stage = int(stage)
        feedback.message = str(message)
        self.server.publish_feedback(feedback)

    def _raise_if_preempted(self):
        if self.server.is_preempt_requested() or self.rospy.is_shutdown():
            raise InteractiveGoalError(
                self.InteractiveGoalResult.PREEMPTED,
                "target request was cancelled",
            )

    def _result(self, success, code, message):
        result = self.InteractiveGoalResult()
        result.success = bool(success)
        result.error_code = int(code)
        result.message = str(message)
        return result

    def _acquire_lock(self):
        lock_directory = os.path.dirname(os.path.abspath(self.lock_path))
        os.makedirs(lock_directory, exist_ok=True)
        lock_handle = open(self.lock_path, "a+", encoding="utf-8")
        try:
            fcntl.flock(lock_handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            lock_handle.close()
            return None
        return lock_handle

    @staticmethod
    def _release_lock(lock_handle):
        if lock_handle is None:
            return
        try:
            fcntl.flock(lock_handle.fileno(), fcntl.LOCK_UN)
        finally:
            lock_handle.close()

    def _snapshot(self):
        with self._condition:
            return (
                self._state,
                self._state_received,
                self._extended_state,
                self._extended_state_received,
                self._odom_received,
                self._planner_status,
                self._planner_status_received,
            )

    def _vehicle_kind(self, auto_arm_if_grounded):
        now = time.monotonic()
        state, state_at, extended, extended_at, _, _, _ = self._snapshot()
        if state is None:
            raise InteractiveGoalError(
                self.InteractiveGoalResult.STATE_STALE,
                "MAVROS state has not been received",
            )
        landed_state = 0 if extended is None else extended.landed_state
        try:
            return vehicle_request_kind(
                connected=bool(state.connected),
                armed=bool(state.armed),
                mode=str(state.mode),
                state_age=now - state_at,
                landed_state=landed_state,
                extended_state_age=(
                    float("inf") if extended is None else now - extended_at
                ),
                state_timeout=self.state_timeout,
                auto_arm_if_grounded=bool(auto_arm_if_grounded),
            )
        except ValueError as exc:
            text = str(exc)
            if "OFFBOARD" in text:
                code = self.InteractiveGoalResult.MODE_REJECTED
            elif "ON_GROUND" in text or "confirmed" in text:
                code = self.InteractiveGoalResult.NOT_ON_GROUND
            elif "connected" in text:
                code = self.InteractiveGoalResult.LINK_UNAVAILABLE
            else:
                code = self.InteractiveGoalResult.STATE_STALE
            raise InteractiveGoalError(code, text)

    def _validate_backend(self, target, constrain_yaw):
        now = time.monotonic()
        _, _, _, _, odom_at, status, status_at = self._snapshot()
        if odom_at <= 0.0 or now - odom_at > self.odom_timeout:
            raise InteractiveGoalError(
                self.InteractiveGoalResult.STATE_STALE,
                "localization odometry is unavailable or stale",
            )
        if status is None or now - status_at > self.planner_status_timeout:
            raise InteractiveGoalError(
                self.InteractiveGoalResult.STATE_STALE,
                "planner status is unavailable or stale",
            )
        if status.state == self.PlannerStatus.FAULT:
            raise InteractiveGoalError(
                self.InteractiveGoalResult.INVALID_GOAL,
                "planner is faulted: {}".format(status.reason or "unknown"),
            )
        if not status.odom_ready or not status.map_ready:
            raise InteractiveGoalError(
                self.InteractiveGoalResult.STATE_STALE,
                "planner odometry or map is not ready",
            )

        planner_goal = self.PlannerGoal()
        planner_goal.header.stamp = self.rospy.Time.now()
        planner_goal.session_id = "ground-station-preflight"
        planner_goal.goal_id = int(time.monotonic_ns() & 0xFFFFFFFFFFFFFFFF)
        planner_goal.action = self.PlannerGoal.PLAN
        planner_goal.goal = target
        planner_goal.goal.header.stamp = planner_goal.header.stamp
        planner_goal.goal.header.frame_id = "world"
        planner_goal.constrain_yaw = bool(constrain_yaw)
        try:
            self.rospy.wait_for_service(
                self.validate_goal_service_name, timeout=self.preflight_timeout
            )
            response = self.validate_goal(planner_goal)
        except Exception as exc:
            raise InteractiveGoalError(
                self.InteractiveGoalResult.INVALID_GOAL,
                "planner goal validation failed: {}".format(exc),
            )
        if not response.valid:
            raise InteractiveGoalError(
                self.InteractiveGoalResult.INVALID_GOAL,
                "planner rejected target: {}".format(
                    response.reason or "unspecified reason"
                ),
            )

    def _stop_child(self):
        child = self._child
        if child is None or child.poll() is not None:
            return
        child.terminate()
        try:
            child.wait(timeout=self._child_stop_timeout)
        except subprocess.TimeoutExpired:
            self.rospy.logerr(
                "Guarded child did not stop after %.1fs; forcing termination",
                self._child_stop_timeout,
            )
            child.kill()
            child.wait(timeout=2.0)

    def _run_command(self, command, monitor_arm=False):
        self._raise_if_preempted()
        self.rospy.loginfo("Running guarded command: %s", " ".join(command))
        self._child_stop_timeout = child_stop_timeout(
            monitor_arm, self.command_timeout
        )
        self._child = subprocess.Popen(command)
        last_stage = None
        try:
            while self._child.poll() is None:
                if self.server.is_preempt_requested() or self.rospy.is_shutdown():
                    self._stop_child()
                    self._raise_if_preempted()
                if monitor_arm:
                    state, _, _, _, _, _, _ = self._snapshot()
                    if state is not None and state.mode == "AUTO.TAKEOFF":
                        stage = self.InteractiveGoalFeedback.TAKING_OFF
                        message = "PX4 AUTO.TAKEOFF is climbing to the safe height"
                    elif state is not None and state.armed:
                        stage = self.InteractiveGoalFeedback.ENTERING_OFFBOARD
                        message = "handing control to verified OFFBOARD"
                    else:
                        stage = self.InteractiveGoalFeedback.ARMING
                        message = "performing guarded arm and takeoff checks"
                    if stage != last_stage:
                        self._feedback(stage, message)
                        last_stage = stage
                time.sleep(0.1)
            return_code = int(self._child.returncode)
            self._raise_if_preempted()
            return return_code
        finally:
            self._child = None
            self._child_stop_timeout = DEFAULT_CHILD_STOP_TIMEOUT

    def _arm_command(self, takeoff_height):
        command = [
            sys.executable,
            "-u",
            self.arm_executor,
            "--takeoff-height",
            str(float(takeoff_height)),
            "--preflight-timeout",
            str(self.preflight_timeout),
            "--command-timeout",
            str(self.command_timeout),
            "--takeoff-timeout",
            str(self.takeoff_timeout),
            "--takeoff-tolerance",
            str(self.takeoff_tolerance),
            "--takeoff-stable-time",
            str(self.takeoff_stable_time),
            "--takeoff-max-vertical-speed",
            str(self.takeoff_max_vertical_speed),
            "--takeoff-altitude-field",
            self.takeoff_altitude_field,
            "--disarmed-prearm-mode",
            self.disarmed_prearm_mode,
            "--odometry-topic",
            self.odometry_topic,
            "--controller-node",
            self.controller_node,
            "--attitude-setpoint-topic",
            self.attitude_setpoint_topic,
        ]
        if self.px4_hover_thrust is not None:
            command.extend(
                ("--px4-hover-thrust", str(float(self.px4_hover_thrust)))
            )
        return command

    def _goal_command(self, coordinates, yaw_degrees):
        command = [
            sys.executable,
            "-u",
            self.goal_executor,
            *(str(value) for value in coordinates),
            "--drone-id",
            "0",
            "--preflight-timeout",
            str(self.preflight_timeout),
            "--odometry-topic",
            self.odometry_topic,
            "--controller-node",
            self.controller_node,
            "--attitude-setpoint-topic",
            self.attitude_setpoint_topic,
        ]
        if yaw_degrees is not None:
            command.extend(("--yaw-deg", str(yaw_degrees)))
        return command

    def _execute(self, goal):
        lock_handle = self._acquire_lock()
        if lock_handle is None:
            self.server.set_aborted(
                self._result(
                    False,
                    self.InteractiveGoalResult.BUSY,
                    "another lifecycle or autonomous command is in progress",
                )
            )
            return
        try:
            self._feedback(
                self.InteractiveGoalFeedback.VALIDATING,
                "validating target and current vehicle state",
            )
            try:
                coordinates, yaw_degrees = validate_goal_request(goal)
            except ValueError as exc:
                raise InteractiveGoalError(
                    self.InteractiveGoalResult.INVALID_GOAL, str(exc)
                )
            self._validate_backend(goal.target, yaw_degrees is not None)
            self._raise_if_preempted()
            request_kind = self._vehicle_kind(goal.auto_arm_if_grounded)

            if request_kind == "disarmed_ground":
                if self._run_command(
                    self._arm_command(goal.takeoff_height), monitor_arm=True
                ) != 0:
                    raise InteractiveGoalError(
                        self.InteractiveGoalResult.ARM_FAILED,
                        "guarded arm/takeoff/OFFBOARD sequence failed",
                    )
                if self._vehicle_kind(False) != "airborne_offboard":
                    raise InteractiveGoalError(
                        self.InteractiveGoalResult.ARM_FAILED,
                        "vehicle did not finish armed in OFFBOARD",
                    )

            self._raise_if_preempted()
            self._feedback(
                self.InteractiveGoalFeedback.SENDING_GOAL,
                "publishing the validated target to the selected planner",
            )
            if self._run_command(
                self._goal_command(coordinates, yaw_degrees)
            ) != 0:
                raise InteractiveGoalError(
                    self.InteractiveGoalResult.GOAL_FAILED,
                    "validated target publication failed; vehicle remains in OFFBOARD",
                )
            self._feedback(
                self.InteractiveGoalFeedback.COMPLETE,
                "target accepted; flight remains in OFFBOARD",
            )
            self.server.set_succeeded(
                self._result(
                    True,
                    self.InteractiveGoalResult.OK,
                    "target accepted by the onboard planner",
                )
            )
        except InteractiveGoalError as exc:
            result = self._result(False, exc.code, str(exc))
            if exc.code == self.InteractiveGoalResult.PREEMPTED:
                self.server.set_preempted(result)
            else:
                self.server.set_aborted(result)
        except Exception as exc:
            self.rospy.logerr("Unexpected interactive-goal failure: %s", exc)
            self.server.set_aborted(
                self._result(
                    False,
                    self.InteractiveGoalResult.GOAL_FAILED,
                    "unexpected onboard error: {}".format(exc),
                )
            )
        finally:
            self._stop_child()
            self._release_lock(lock_handle)


def main():
    import rospy

    rospy.init_node("interactive_goal_server")
    InteractiveGoalServer()
    rospy.spin()


if __name__ == "__main__":
    main()
