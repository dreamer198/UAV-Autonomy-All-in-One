#!/usr/bin/env python3
"""Guarded takeoff and landing commands for the embedded RViz toolbar.

The target action remains deliberately separate.  TAKEOFF reuses the shared
arm executor and finishes only after verified OFFBOARD hold; LAND requests PX4
AUTO.LAND and never force-disarms the vehicle.  Both commands share the same
lifecycle lock as interactive goals and command-line flight operations.
"""

import fcntl
import math
import os
import subprocess
import sys
import threading
import time


COMMAND_TAKEOFF = 1
COMMAND_LAND = 2
ON_GROUND = 1
IN_AIR = 2
TAKING_OFF = 3
LANDING = 4
MIN_TAKEOFF_HEIGHT = 0.5
MAX_TAKEOFF_HEIGHT = 2.5
DEFAULT_CHILD_STOP_TIMEOUT = 3.0
LANDABLE_AUTONOMOUS_MODES = (
    "OFFBOARD",
    "AUTO.TAKEOFF",
    "AUTO.LOITER",
    "AUTO.LAND",
)


class FlightRequestError(ValueError):
    """A ROS-independent request/state validation failure."""

    def __init__(self, reason, message):
        super().__init__(message)
        self.reason = str(reason)


class FlightCommandError(RuntimeError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = int(code)


def finite_number(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError, OverflowError):
        return False


def validate_command_request(command, takeoff_height):
    """Validate the action payload without importing generated ROS messages."""
    try:
        command = int(command)
    except (TypeError, ValueError, OverflowError):
        raise FlightRequestError("invalid_request", "unsupported flight command")
    if command not in (COMMAND_TAKEOFF, COMMAND_LAND):
        raise FlightRequestError("invalid_request", "unsupported flight command")
    if isinstance(takeoff_height, bool):
        raise FlightRequestError(
            "invalid_request", "takeoff height field must be a finite number"
        )
    if not finite_number(takeoff_height):
        raise FlightRequestError(
            "invalid_request", "takeoff height field must be finite"
        )
    if command == COMMAND_TAKEOFF:
        height = float(takeoff_height)
        if not MIN_TAKEOFF_HEIGHT <= height <= MAX_TAKEOFF_HEIGHT:
            raise FlightRequestError(
                "invalid_request",
                "takeoff height must be within [{:.1f}, {:.1f}] m".format(
                    MIN_TAKEOFF_HEIGHT, MAX_TAKEOFF_HEIGHT
                ),
            )
        return command, height
    # LAND has no height parameter.  Ignoring it keeps the wire format compact
    # without assigning safety meaning to an unused client value.
    return command, None


def vehicle_request_kind(
    *,
    command,
    connected,
    armed,
    mode,
    state_age,
    landed_state,
    extended_state_age,
    state_timeout,
):
    """Classify a command using fresh MAVROS state and landed-state evidence."""
    if (
        not finite_number(state_timeout)
        or float(state_timeout) <= 0.0
        or not finite_number(state_age)
        or float(state_age) < 0.0
        or float(state_age) > float(state_timeout)
    ):
        raise FlightRequestError("state_stale", "MAVROS state is stale")
    if not connected:
        raise FlightRequestError(
            "link_unavailable", "MAVROS is not connected to PX4"
        )

    if command == COMMAND_TAKEOFF:
        if armed:
            raise FlightRequestError(
                "state_rejected", "takeoff requires a disarmed vehicle"
            )
        if (
            not finite_number(extended_state_age)
            or float(extended_state_age) < 0.0
            or float(extended_state_age) > float(state_timeout)
        ):
            raise FlightRequestError(
                "state_stale", "MAVROS extended state is stale"
            )
        if int(landed_state) != ON_GROUND:
            raise FlightRequestError(
                "state_rejected", "vehicle is not reliably ON_GROUND"
            )
        return "disarmed_ground"

    if command != COMMAND_LAND:
        raise FlightRequestError("invalid_request", "unsupported flight command")
    if not armed:
        raise FlightRequestError(
            "state_rejected", "landing requires an armed airborne vehicle"
        )
    if (
        not finite_number(extended_state_age)
        or float(extended_state_age) < 0.0
        or float(extended_state_age) > float(state_timeout)
    ):
        raise FlightRequestError(
            "state_stale", "MAVROS extended state is stale"
        )
    if int(landed_state) not in (IN_AIR, TAKING_OFF, LANDING):
        raise FlightRequestError(
            "state_rejected", "vehicle is not reliably airborne"
        )
    if str(mode) not in LANDABLE_AUTONOMOUS_MODES:
        raise FlightRequestError(
            "state_rejected",
            "refusing to override pilot/manual mode {} with AUTO.LAND".format(
                mode or "unknown"
            ),
        )
    return "armed_airborne"


def takeoff_handoff_ready(*, connected, armed, mode, state_age, state_timeout):
    """Return whether the arm executor left a fresh, verified OFFBOARD state."""
    return (
        finite_number(state_age)
        and finite_number(state_timeout)
        and float(state_timeout) > 0.0
        and float(state_age) >= 0.0
        and float(state_age) <= float(state_timeout)
        and bool(connected)
        and bool(armed)
        and str(mode) == "OFFBOARD"
    )


def child_stop_timeout(monitor_arm, command_timeout):
    """Allow interrupted takeoff to finish the executor's safe recovery."""
    if not monitor_arm:
        return DEFAULT_CHILD_STOP_TIMEOUT
    return max(
        DEFAULT_CHILD_STOP_TIMEOUT,
        2.0 * float(command_timeout) + 2.0,
    )


class FlightCommandServer:
    def __init__(self):
        import actionlib
        import rospy
        import rospkg
        from mavros_msgs.msg import ExtendedState, State
        from mavros_msgs.srv import SetMode
        from sim2real_planning_msgs.msg import (
            FlightCommandAction,
            FlightCommandFeedback,
            FlightCommandResult,
        )

        self.rospy = rospy
        self.FlightCommandFeedback = FlightCommandFeedback
        self.FlightCommandResult = FlightCommandResult
        self._condition = threading.Condition()
        self._state = None
        self._state_received = 0.0
        self._extended_state = None
        self._extended_state_received = 0.0
        self._child = None
        self._child_stop_timeout = DEFAULT_CHILD_STOP_TIMEOUT

        self.action_name = rospy.get_param(
            "~action_name", "/ground_station/flight_command"
        )
        self.lock_path = rospy.get_param(
            "~lock_path", "/root/tmp/real.lifecycle.lock"
        )
        self.state_timeout = float(rospy.get_param("~state_timeout", 3.0))
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
        self.odometry_topic = rospy.get_param(
            "~odometry_topic", "/localization/odom"
        )
        self.controller_node = rospy.get_param(
            "~controller_node", "/se3_controller_node"
        )
        self.attitude_setpoint_topic = rospy.get_param(
            "~attitude_setpoint_topic", "/mavros/setpoint_raw/attitude"
        )
        self.land_retry_interval = float(
            rospy.get_param("~land_retry_interval", 0.2)
        )
        self._validate_parameters()

        package_path = rospkg.RosPack().get_path("sim2real_common")
        self.arm_executor = os.path.join(package_path, "scripts", "arm_executor.py")
        if not os.path.isfile(self.arm_executor):
            raise RuntimeError(
                "shared arm executor is missing: {}".format(self.arm_executor)
            )

        rospy.Subscriber(
            "/mavros/state", State, self._state_callback, queue_size=1
        )
        rospy.Subscriber(
            "/mavros/extended_state",
            ExtendedState,
            self._extended_state_callback,
            queue_size=1,
        )
        self.set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.server = actionlib.SimpleActionServer(
            self.action_name,
            FlightCommandAction,
            execute_cb=self._execute,
            auto_start=False,
        )
        self.server.start()
        rospy.on_shutdown(self._stop_child)
        rospy.loginfo("Guarded flight-command action ready: %s", self.action_name)

    def _validate_parameters(self):
        positive_parameters = (
            ("state_timeout", self.state_timeout),
            ("preflight_timeout", self.preflight_timeout),
            ("command_timeout", self.command_timeout),
            ("takeoff_timeout", self.takeoff_timeout),
            ("takeoff_tolerance", self.takeoff_tolerance),
            ("takeoff_stable_time", self.takeoff_stable_time),
            ("takeoff_max_vertical_speed", self.takeoff_max_vertical_speed),
            ("land_retry_interval", self.land_retry_interval),
        )
        for name, value in positive_parameters:
            if not finite_number(value) or float(value) <= 0.0:
                raise ValueError("~{} must be finite and positive".format(name))
        if self.takeoff_altitude_field not in ("relative", "local", "auto"):
            raise ValueError("invalid ~takeoff_altitude_field")
        if self.disarmed_prearm_mode not in ("STABILIZED", "AUTO.LOITER"):
            raise ValueError("invalid ~disarmed_prearm_mode")
        if self.px4_hover_thrust is not None and (
            not finite_number(self.px4_hover_thrust)
            or not 0.0 < float(self.px4_hover_thrust) <= 1.0
        ):
            raise ValueError("~px4_hover_thrust must be within (0, 1]")

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

    def _snapshot(self):
        with self._condition:
            return (
                self._state,
                self._state_received,
                self._extended_state,
                self._extended_state_received,
            )

    def _feedback(self, stage, message):
        feedback = self.FlightCommandFeedback()
        feedback.stage = int(stage)
        feedback.message = str(message)
        self.server.publish_feedback(feedback)

    def _raise_if_preempted(self):
        if self.server.is_preempt_requested() or self.rospy.is_shutdown():
            raise FlightCommandError(
                self.FlightCommandResult.PREEMPTED,
                "flight command was cancelled; no flight-mode rollback was requested",
            )

    def _result(self, success, code, message):
        result = self.FlightCommandResult()
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

    def _map_request_error(self, exc):
        codes = {
            "invalid_request": self.FlightCommandResult.INVALID_REQUEST,
            "link_unavailable": self.FlightCommandResult.LINK_UNAVAILABLE,
            "state_stale": self.FlightCommandResult.STATE_STALE,
            "state_rejected": self.FlightCommandResult.STATE_REJECTED,
        }
        return FlightCommandError(
            codes.get(exc.reason, self.FlightCommandResult.STATE_REJECTED),
            str(exc),
        )

    def _vehicle_kind(self, command):
        now = time.monotonic()
        state, state_at, extended, extended_at = self._snapshot()
        if state is None:
            raise FlightCommandError(
                self.FlightCommandResult.STATE_STALE,
                "MAVROS state has not been received",
            )
        try:
            return vehicle_request_kind(
                command=command,
                connected=bool(state.connected),
                armed=bool(state.armed),
                mode=str(state.mode),
                state_age=now - state_at,
                landed_state=0 if extended is None else extended.landed_state,
                extended_state_age=(
                    float("inf") if extended is None else now - extended_at
                ),
                state_timeout=self.state_timeout,
            )
        except FlightRequestError as exc:
            raise self._map_request_error(exc)

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

    def _run_takeoff(self, takeoff_height):
        self._raise_if_preempted()
        self._child_stop_timeout = child_stop_timeout(
            True, self.command_timeout
        )
        self._child = subprocess.Popen(self._arm_command(takeoff_height))
        last_stage = None
        try:
            while self._child.poll() is None:
                if self.server.is_preempt_requested() or self.rospy.is_shutdown():
                    self._stop_child()
                    self._raise_if_preempted()
                state, _, _, _ = self._snapshot()
                if state is not None and state.mode == "AUTO.TAKEOFF":
                    stage = self.FlightCommandFeedback.TAKING_OFF
                    message = "PX4 AUTO.TAKEOFF is climbing to the safe height"
                elif state is not None and state.armed:
                    stage = self.FlightCommandFeedback.ENTERING_OFFBOARD
                    message = "handing control to verified OFFBOARD hold"
                else:
                    stage = self.FlightCommandFeedback.ARMING
                    message = "performing guarded arm and takeoff checks"
                if stage != last_stage:
                    self._feedback(stage, message)
                    last_stage = stage
                time.sleep(0.1)
            return_code = int(self._child.returncode)
            self._raise_if_preempted()
            if return_code != 0:
                raise FlightCommandError(
                    self.FlightCommandResult.TAKEOFF_FAILED,
                    "guarded arm/takeoff/OFFBOARD sequence failed",
                )
        finally:
            self._child = None
            self._child_stop_timeout = DEFAULT_CHILD_STOP_TIMEOUT

        now = time.monotonic()
        state, state_at, _, _ = self._snapshot()
        if state is None or not takeoff_handoff_ready(
            connected=bool(state.connected),
            armed=bool(state.armed),
            mode=str(state.mode),
            state_age=now - state_at,
            state_timeout=self.state_timeout,
        ):
            raise FlightCommandError(
                self.FlightCommandResult.TAKEOFF_FAILED,
                "vehicle did not finish takeoff armed in fresh OFFBOARD hold",
            )

    def _request_land(self):
        self._raise_if_preempted()
        try:
            self.rospy.wait_for_service(
                "/mavros/set_mode", timeout=self.preflight_timeout
            )
        except self.rospy.ROSException as exc:
            raise FlightCommandError(
                self.FlightCommandResult.LAND_FAILED,
                "MAVROS set-mode service is unavailable: {}".format(exc),
            )

        self._feedback(
            self.FlightCommandFeedback.REQUESTING_LAND,
            "requesting and verifying PX4 AUTO.LAND",
        )
        deadline = time.monotonic() + self.command_timeout
        last_request_at = 0.0
        while True:
            now = time.monotonic()
            state, state_at, _, _ = self._snapshot()
            if state is None or now - state_at > self.state_timeout:
                raise FlightCommandError(
                    self.FlightCommandResult.STATE_STALE,
                    "MAVROS state became unavailable or stale during landing request",
                )
            if not state.connected:
                raise FlightCommandError(
                    self.FlightCommandResult.LINK_UNAVAILABLE,
                    "MAVROS disconnected before AUTO.LAND was confirmed",
                )
            # Once PX4 reports AUTO.LAND, cancellation cannot safely undo it and
            # the command is complete.  Deliberately check this before preempt.
            if state.armed and state.mode == "AUTO.LAND":
                return
            if not state.armed:
                raise FlightCommandError(
                    self.FlightCommandResult.LAND_FAILED,
                    "vehicle disarmed before AUTO.LAND was confirmed",
                )
            if state.mode not in LANDABLE_AUTONOMOUS_MODES:
                raise FlightCommandError(
                    self.FlightCommandResult.STATE_REJECTED,
                    "pilot/manual mode {} took control; AUTO.LAND was not forced".format(
                        state.mode or "unknown"
                    ),
                )
            self._raise_if_preempted()
            if now >= deadline:
                raise FlightCommandError(
                    self.FlightCommandResult.LAND_FAILED,
                    "PX4 did not enter AUTO.LAND within {:.1f}s".format(
                        self.command_timeout
                    ),
                )
            if now - last_request_at >= self.land_retry_interval:
                try:
                    response = self.set_mode(base_mode=0, custom_mode="AUTO.LAND")
                    if not response.mode_sent:
                        self.rospy.logerr_throttle(
                            1.0, "PX4 rejected the AUTO.LAND mode request"
                        )
                except self.rospy.ServiceException as exc:
                    self.rospy.logerr_throttle(
                        1.0, "MAVROS AUTO.LAND request failed: %s", exc
                    )
                last_request_at = now
            with self._condition:
                self._condition.wait(min(0.1, max(0.0, deadline - now)))

    def _execute(self, goal):
        lock_handle = self._acquire_lock()
        if lock_handle is None:
            self.server.set_aborted(
                self._result(
                    False,
                    self.FlightCommandResult.BUSY,
                    "another lifecycle or autonomous command is in progress",
                )
            )
            return
        try:
            self._feedback(
                self.FlightCommandFeedback.VALIDATING,
                "validating command and current vehicle state",
            )
            try:
                command, takeoff_height = validate_command_request(
                    goal.command, goal.takeoff_height
                )
            except FlightRequestError as exc:
                raise self._map_request_error(exc)
            self._raise_if_preempted()
            self._vehicle_kind(command)

            if command == COMMAND_TAKEOFF:
                self._run_takeoff(takeoff_height)
                message = "takeoff complete; vehicle is hovering in armed OFFBOARD"
            else:
                self._request_land()
                message = "PX4 AUTO.LAND is active; no forced disarm was requested"

            self._feedback(self.FlightCommandFeedback.COMPLETE, message)
            self.server.set_succeeded(
                self._result(True, self.FlightCommandResult.OK, message)
            )
        except FlightCommandError as exc:
            result = self._result(False, exc.code, str(exc))
            if exc.code == self.FlightCommandResult.PREEMPTED:
                self.server.set_preempted(result)
            else:
                self.server.set_aborted(result)
        except Exception as exc:
            self.rospy.logerr("Unexpected flight-command failure: %s", exc)
            command = getattr(goal, "command", 0)
            code = (
                self.FlightCommandResult.LAND_FAILED
                if command == COMMAND_LAND
                else self.FlightCommandResult.TAKEOFF_FAILED
            )
            self.server.set_aborted(
                self._result(
                    False,
                    code,
                    "unexpected onboard error: {}".format(exc),
                )
            )
        finally:
            self._stop_child()
            self._release_lock(lock_handle)


def main():
    import rospy

    rospy.init_node("flight_command_server")
    FlightCommandServer()
    rospy.spin()


if __name__ == "__main__":
    main()
