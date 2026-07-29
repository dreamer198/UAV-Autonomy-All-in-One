#!/usr/bin/env python3
"""Shared end-to-end PX4 mission lifecycle for simulation and real flight."""

import argparse
import math
import signal
import sys
import threading
import time

from waypoint_mission import (
    EXIT_MANUAL_TAKEOVER,
    EXIT_MISSION_FAILED,
    EXIT_SUCCESS,
    MissionConfigError,
    LOCALIZATION_FAULT_PARAM,
    WaypointMission,
    load_mission_config,
    localization_fault_reason,
)

MAV_STATE_FLIGHT_TERMINATION = 8


class FlightDirectorError(RuntimeError):
    def __init__(self, message, code=EXIT_MISSION_FAILED):
        super().__init__(message)
        self.code = code


def native_takeoff_target(height, acceptance_radius):
    values = (height, acceptance_radius)
    if not all(math.isfinite(value) for value in values):
        raise FlightDirectorError(
            "takeoff height and NAV_MC_ALT_RAD must be finite"
        )
    if height <= 0.0 or acceptance_radius < 0.0:
        raise FlightDirectorError(
            "takeoff height must be positive and NAV_MC_ALT_RAD non-negative"
        )
    # MIS_TAKEOFF_ALT is the actual altitude setpoint on the deployed PX4.
    # NAV_MC_ALT_RAD is never added to the requested height.
    return height


def temporary_takeoff_acceptance_radius(current_radius, altitude_tolerance):
    values = (current_radius, altitude_tolerance)
    if not all(math.isfinite(value) and value > 0.0 for value in values):
        raise FlightDirectorError(
            "takeoff acceptance radius inputs must be finite and positive"
        )
    return min(current_radius, altitude_tolerance)


def native_takeoff_handoff_ready(
    mode,
    altitude,
    vertical_speed,
    height,
    tolerance,
    max_vertical_speed,
):
    """Return whether PX4 has completed and settled its native takeoff."""
    values = (altitude, vertical_speed, height, tolerance, max_vertical_speed)
    if not all(
        value is not None and math.isfinite(float(value))
        for value in values
    ):
        return False
    return (
        mode in ("AUTO.TAKEOFF", "AUTO.LOITER")
        and float(height) - float(tolerance)
        <= float(altitude)
        <= float(height) + float(tolerance)
        and abs(float(vertical_speed)) <= float(max_vertical_speed)
    )


def preflight_position_stream_ready(
    armed, setpoint_count, setpoint_age, odom_timeout, required_samples=10
):
    """An armed vehicle no longer needs the pre-OFFBOARD position stream."""
    if armed:
        return True
    return (
        setpoint_count >= required_samples
        and setpoint_age <= odom_timeout
    )


def disarmed_mode_requires_stabilized(mode):
    """Return whether a disarmed PX4 should leave an autonomous mode."""
    return mode == "OFFBOARD" or bool(mode and mode.startswith("AUTO."))


def disarmed_mode_requires_reset(mode, target_mode):
    """Return whether PX4 must enter the configured safe pre-arm mode."""
    if target_mode == "STABILIZED":
        return disarmed_mode_requires_stabilized(mode)
    return mode != target_mode


def px4_flight_termination_active(system_status):
    return int(system_status) == MAV_STATE_FLIGHT_TERMINATION


def flight_director_recovery_mode(reason):
    """Choose a recoverable PX4 native mode without relying on OFFBOARD."""
    if "localization" in str(reason).lower():
        return "AUTO.LAND"
    return "AUTO.LOITER"


class SharedMissionExecutor:
    """One mission state machine used unchanged by SITL and real flight."""

    def __init__(self, rospy, config, args):
        import rosnode
        from geometry_msgs.msg import PoseStamped
        from mavros_msgs.msg import Altitude, AttitudeTarget, ParamValue, State
        from mavros_msgs.srv import CommandBool, ParamGet, ParamSet, SetMode
        from nav_msgs.msg import Odometry

        self.rospy = rospy
        self.rosnode = rosnode
        self.ParamValue = ParamValue
        self.args = args
        self.config = config
        self.condition = threading.Condition()
        self.abort_requested = False
        self.state = None
        self.state_received_at = 0.0
        self.relative_altitude = None
        self.altitude_received_at = 0.0
        self.vertical_velocity = None
        self.odom_received_at = 0.0
        self.position_setpoint_count = 0
        self.position_setpoint_received_at = 0.0
        self.attitude_setpoint_count = 0
        self.original_altitude_acceptance_radius = None
        self.altitude_acceptance_radius_changed = False

        # Warm the exact waypoint runner that will execute the mission while
        # PX4 is still disarmed/climbing. This removes the post-takeoff ROS
        # publisher/subscriber startup pause.
        self.runner = WaypointMission(
            config,
            args.drone_id,
            state_topic=args.state_topic,
            odometry_topic=args.odometry_topic,
        )

        rospy.Subscriber(
            args.state_topic, State, self._state_callback, queue_size=1
        )
        rospy.Subscriber(
            args.altitude_topic,
            Altitude,
            self._altitude_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            args.odometry_topic,
            Odometry,
            self._odom_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            "/mavros/setpoint_position/local",
            PoseStamped,
            self._position_setpoint_callback,
            queue_size=20,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            "/mavros/setpoint_raw/attitude",
            AttitudeTarget,
            self._attitude_setpoint_callback,
            queue_size=20,
            tcp_nodelay=True,
        )

        self.arm_vehicle = rospy.ServiceProxy(
            "/mavros/cmd/arming", CommandBool
        )
        self.set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.param_get = rospy.ServiceProxy("/mavros/param/get", ParamGet)
        self.param_set = rospy.ServiceProxy("/mavros/param/set", ParamSet)

    def _state_callback(self, message):
        with self.condition:
            self.state = message
            self.state_received_at = time.monotonic()
            self.condition.notify_all()

    def _altitude_callback(self, message):
        with self.condition:
            self.relative_altitude = message.relative
            self.altitude_received_at = time.monotonic()
            self.condition.notify_all()

    def _odom_callback(self, message):
        with self.condition:
            vertical_velocity = float(message.twist.twist.linear.z)
            self.vertical_velocity = (
                vertical_velocity if math.isfinite(vertical_velocity) else None
            )
            self.odom_received_at = time.monotonic()
            self.condition.notify_all()

    def _position_setpoint_callback(self, _message):
        with self.condition:
            self.position_setpoint_count += 1
            self.position_setpoint_received_at = time.monotonic()
            self.condition.notify_all()

    def _attitude_setpoint_callback(self, _message):
        with self.condition:
            self.attitude_setpoint_count += 1
            self.condition.notify_all()

    def request_abort(self):
        with self.condition:
            self.abort_requested = True
            self.condition.notify_all()
        self.runner.request_abort()

    def _check_abort(self):
        if self.abort_requested or self.rospy.is_shutdown():
            raise FlightDirectorError("mission executor was interrupted")

    def _check_localization_interlock(self):
        try:
            value = self.rospy.get_param(LOCALIZATION_FAULT_PARAM, "")
        except Exception as exc:
            raise FlightDirectorError(
                "localization safety interlock could not be read: {}".format(
                    exc
                )
            )
        reason = localization_fault_reason(value)
        if reason:
            raise FlightDirectorError(
                "localization safety interlock is latched: {}. Restart the "
                "complete simulation/real stack before another mission".format(
                    reason
                )
            )

    def _check_localization_health(self):
        """Fail closed when the interlock or live odometry is unavailable."""
        self._check_localization_interlock()
        now = time.monotonic()
        with self.condition:
            odom_received_at = self.odom_received_at
        age = now - odom_received_at
        if odom_received_at <= 0.0 or age > self.config["odom_timeout"]:
            raise FlightDirectorError(
                "localization odometry is unavailable or stale "
                "(age {:.2f}s, limit {:.2f}s)".format(
                    age, self.config["odom_timeout"]
                )
            )

    def _wait_for_services(self):
        for name in (
            "/mavros/cmd/arming",
            "/mavros/set_mode",
            "/mavros/param/get",
            "/mavros/param/set",
        ):
            try:
                self.rospy.wait_for_service(
                    name, timeout=self.args.preflight_timeout
                )
            except self.rospy.ROSException as exc:
                raise FlightDirectorError(
                    "required service {} is unavailable: {}".format(name, exc)
                )

    def _wait_for_preflight_data(self):
        try:
            nodes = self.rosnode.get_node_names()
        except Exception as exc:
            raise FlightDirectorError("cannot query ROS nodes: {}".format(exc))
        if "/se3_controller_node" not in nodes:
            raise FlightDirectorError("SE3 controller is not running")

        deadline = time.monotonic() + self.args.preflight_timeout
        while True:
            self._check_abort()
            self._check_localization_interlock()
            now = time.monotonic()
            with self.condition:
                state = self.state
                state_received_at = self.state_received_at
                relative_altitude = self.relative_altitude
                altitude_received_at = self.altitude_received_at
                odom_received_at = self.odom_received_at
                position_setpoint_count = self.position_setpoint_count
                position_setpoint_received_at = (
                    self.position_setpoint_received_at
                )
            state_is_fresh = (
                state is not None
                and now - state_received_at <= self.config["state_timeout"]
            )
            altitude_is_fresh = (
                relative_altitude is not None
                and now - altitude_received_at <= self.args.altitude_timeout
            )
            odom_is_fresh = (
                odom_received_at > 0.0
                and now - odom_received_at <= self.config["odom_timeout"]
            )
            position_stream_ready = preflight_position_stream_ready(
                state.armed if state_is_fresh else False,
                position_setpoint_count,
                now - position_setpoint_received_at,
                self.config["odom_timeout"],
            )
            # Subscribers are connected asynchronously when this short-lived
            # mission process starts.  If PX4 is already armed, /mavros/state
            # can win that startup race by a few milliseconds and arrive
            # before the first /localization/odom callback.  Wait for the
            # initial odometry sample until the normal preflight deadline, but
            # still fail immediately if a stream that has actually been seen
            # becomes stale while armed.
            if (
                state_is_fresh
                and state.armed
                and odom_received_at > 0.0
                and not odom_is_fresh
            ):
                raise FlightDirectorError(
                    "localization odometry is unavailable or stale while "
                    "the vehicle is armed"
                )
            ready = (
                state_is_fresh
                and altitude_is_fresh
                and math.isfinite(float(relative_altitude))
                and odom_is_fresh
                and position_stream_ready
            )
            if ready:
                break
            remaining = deadline - now
            if remaining <= 0.0:
                missing = []
                if not state_is_fresh:
                    missing.append("fresh MAVROS state")
                if (
                    not altitude_is_fresh
                    or not math.isfinite(float(relative_altitude))
                ):
                    missing.append("fresh relative altitude")
                if not odom_is_fresh:
                    missing.append("fresh localization")
                if not position_stream_ready:
                    missing.append("ten fresh position hold setpoints")
                raise FlightDirectorError(
                    "mission preflight was not ready within {:.1f}s; "
                    "missing: {}".format(
                        self.args.preflight_timeout,
                        ", ".join(missing) or "unknown condition",
                    )
                )
            with self.condition:
                self.condition.wait(min(remaining, 0.1))
        if not state.connected:
            raise FlightDirectorError("MAVROS is not connected to PX4")
        if (
            not state.armed
            and px4_flight_termination_active(state.system_status)
        ):
            raise FlightDirectorError(
                "PX4 reports FLIGHT_TERMINATION (system_status=8); release "
                "the RC kill switch and clear the termination state before "
                "running a mission"
            )

    def _state_snapshot(self):
        with self.condition:
            return self.state, self.relative_altitude

    def _state_is_fresh(self, now=None):
        if now is None:
            now = time.monotonic()
        with self.condition:
            return (
                self.state is not None
                and now - self.state_received_at
                <= self.config["state_timeout"]
            )

    def _altitude_is_fresh(self, now=None):
        if now is None:
            now = time.monotonic()
        with self.condition:
            return (
                self.relative_altitude is not None
                and now - self.altitude_received_at
                <= self.args.altitude_timeout
            )

    def _reset_disarmed_mode(self):
        target_mode = self.args.disarmed_prearm_mode
        self.rospy.logwarn(
            "PX4 is disarmed in %s; entering %s before takeoff.",
            self.state.mode,
            target_mode,
        )
        deadline = time.monotonic() + self.args.command_timeout
        while time.monotonic() < deadline:
            self._check_abort()
            self._check_localization_health()
            state, _ = self._state_snapshot()
            if (
                state is None
                or not self._state_is_fresh()
                or not state.connected
            ):
                raise FlightDirectorError("PX4 state became unavailable")
            if state.armed:
                raise FlightDirectorError(
                    "PX4 armed unexpectedly while resetting its disarmed mode"
                )
            if state.mode == target_mode:
                return
            if (
                target_mode == "STABILIZED"
                and not disarmed_mode_requires_stabilized(state.mode)
            ):
                self.rospy.loginfo(
                    "PX4 is already disarmed in pilot mode %s; leaving that "
                    "mode unchanged.",
                    state.mode or "unknown",
                )
                return
            try:
                self.set_mode(base_mode=0, custom_mode=target_mode)
            except self.rospy.ServiceException:
                pass
            with self.condition:
                self.condition.wait(0.1)
        raise FlightDirectorError(
            "PX4 did not enter disarmed {} within {:.1f}s".format(
                target_mode, self.args.command_timeout
            )
        )

    def _configure_takeoff(self):
        if self.args.px4_hover_thrust is not None:
            try:
                hover_response = self.param_set(
                    param_id="MPC_THR_HOVER",
                    value=self.ParamValue(
                        integer=0, real=self.args.px4_hover_thrust
                    ),
                )
            except self.rospy.ServiceException as exc:
                raise FlightDirectorError(
                    "unable to configure MPC_THR_HOVER: {}".format(exc)
                )
            if (
                not hover_response.success
                or abs(
                    float(hover_response.value.real)
                    - self.args.px4_hover_thrust
                )
                > 1e-3
            ):
                raise FlightDirectorError(
                    "PX4 rejected MPC_THR_HOVER={}".format(
                        self.args.px4_hover_thrust
                    )
                )
            self.rospy.loginfo(
                "PX4 native controller hover thrust set to %.3f.",
                self.args.px4_hover_thrust,
            )

        try:
            radius_response = self.param_get(param_id="NAV_MC_ALT_RAD")
        except self.rospy.ServiceException as exc:
            raise FlightDirectorError(
                "unable to read NAV_MC_ALT_RAD: {}".format(exc)
            )
        if not radius_response.success:
            raise FlightDirectorError("PX4 rejected the NAV_MC_ALT_RAD read")
        acceptance_radius = float(radius_response.value.real)
        temporary_radius = temporary_takeoff_acceptance_radius(
            acceptance_radius, self.args.takeoff_tolerance
        )
        target = native_takeoff_target(
            self.config["takeoff_height"], acceptance_radius
        )
        self.original_altitude_acceptance_radius = acceptance_radius
        self.altitude_acceptance_radius_changed = (
            abs(temporary_radius - acceptance_radius) > 1e-3
        )
        try:
            radius_set_response = self.param_set(
                param_id="NAV_MC_ALT_RAD",
                value=self.ParamValue(integer=0, real=temporary_radius),
            )
        except self.rospy.ServiceException as exc:
            raise FlightDirectorError(
                "unable to configure temporary NAV_MC_ALT_RAD: {}".format(exc)
            )
        if (
            not radius_set_response.success
            or abs(float(radius_set_response.value.real) - temporary_radius)
            > 1e-3
        ):
            raise FlightDirectorError(
                "PX4 rejected temporary NAV_MC_ALT_RAD={}".format(
                    temporary_radius
                )
            )

        try:
            altitude_response = self.param_set(
                param_id="MIS_TAKEOFF_ALT",
                value=self.ParamValue(integer=0, real=target),
            )
            action_response = self.param_set(
                param_id="COM_TAKEOFF_ACT",
                value=self.ParamValue(integer=0, real=0.0),
            )
        except self.rospy.ServiceException as exc:
            raise FlightDirectorError(
                "unable to configure PX4 native takeoff: {}".format(exc)
            )
        if (
            not altitude_response.success
            or abs(float(altitude_response.value.real) - target) > 1e-3
        ):
            raise FlightDirectorError("PX4 rejected MIS_TAKEOFF_ALT={}".format(target))
        if not action_response.success or action_response.value.integer != 0:
            raise FlightDirectorError("PX4 rejected COM_TAKEOFF_ACT=0")
        self.rospy.loginfo(
            "Native takeoff configured: desired=%.2f m, temporary "
            "NAV_MC_ALT_RAD=%.2f m (original %.2f m), "
            "MIS_TAKEOFF_ALT=%.2f m.",
            self.config["takeoff_height"],
            temporary_radius,
            acceptance_radius,
            target,
        )

    def _restore_altitude_acceptance_radius(self):
        if not self.altitude_acceptance_radius_changed:
            return
        original = self.original_altitude_acceptance_radius
        try:
            response = self.param_set(
                param_id="NAV_MC_ALT_RAD",
                value=self.ParamValue(integer=0, real=original),
            )
        except Exception as exc:
            self.rospy.logerr(
                "Unable to restore NAV_MC_ALT_RAD=%.2f: %s", original, exc
            )
            return
        if (
            not response.success
            or abs(float(response.value.real) - original) > 1e-3
        ):
            self.rospy.logerr(
                "PX4 did not restore NAV_MC_ALT_RAD to %.2f; inspect the "
                "parameter before another flight.",
                original,
            )
            return
        self.altitude_acceptance_radius_changed = False
        self.rospy.loginfo("Restored NAV_MC_ALT_RAD=%.2f.", original)

    def _arm_and_start_takeoff(self):
        deadline = time.monotonic() + self.args.command_timeout
        accepted = False
        while time.monotonic() < deadline:
            self._check_abort()
            self._check_localization_health()
            try:
                response = self.arm_vehicle(value=True)
            except self.rospy.ServiceException:
                response = None
            if response is not None and response.success:
                accepted = True
                break
            time.sleep(0.1)
        if not accepted:
            raise FlightDirectorError(
                "PX4 did not accept arming within {:.1f}s".format(
                    self.args.command_timeout
                )
            )

        deadline = time.monotonic() + self.args.command_timeout
        while time.monotonic() < deadline:
            self._check_abort()
            self._check_localization_health()
            state, _ = self._state_snapshot()
            if (
                state is not None
                and self._state_is_fresh()
                and state.armed
                and state.mode == "AUTO.TAKEOFF"
            ):
                self.rospy.loginfo("PX4 is armed in AUTO.TAKEOFF.")
                return
            try:
                self.set_mode(base_mode=0, custom_mode="AUTO.TAKEOFF")
            except self.rospy.ServiceException:
                pass
            with self.condition:
                self.condition.wait(0.1)
        raise FlightDirectorError(
            "PX4 did not remain armed in AUTO.TAKEOFF within {:.1f}s".format(
                self.args.command_timeout
            )
        )

    def _wait_for_native_takeoff_settle(self):
        deadline = time.monotonic() + self.args.takeoff_timeout
        started = time.monotonic()
        height_reached = False
        stable_since = None
        while time.monotonic() < deadline:
            self._check_abort()
            self._check_localization_health()
            state, altitude = self._state_snapshot()
            if (
                state is None
                or not self._state_is_fresh()
                or not state.connected
                or not state.armed
            ):
                raise FlightDirectorError(
                    "PX4 state became stale, disconnected or disarmed during "
                    "AUTO.TAKEOFF"
                )
            if state.mode not in ("AUTO.TAKEOFF", "AUTO.LOITER"):
                raise FlightDirectorError(
                    "flight mode changed to {} during takeoff".format(
                        state.mode or "unknown"
                    ),
                    EXIT_MANUAL_TAKEOVER,
                )
            if (
                altitude is not None
                and math.isfinite(float(altitude))
                and altitude
                >= self.config["takeoff_height"] - self.args.takeoff_tolerance
            ):
                if not height_reached:
                    height_reached = True
                    self.rospy.loginfo(
                        "Native takeoff reached %.2f m after %.2f s; waiting "
                        "for PX4 native-mode altitude and vertical settling "
                        "before OFFBOARD.",
                        altitude,
                        time.monotonic() - started,
                    )

            now = time.monotonic()
            with self.condition:
                vertical_velocity = self.vertical_velocity
                odom_is_fresh = (
                    now - self.odom_received_at
                    <= self.config["odom_timeout"]
                )
                altitude_is_fresh = (
                    self.relative_altitude is not None
                    and now - self.altitude_received_at
                    <= self.args.altitude_timeout
                )
            ready = (
                odom_is_fresh
                and altitude_is_fresh
                and native_takeoff_handoff_ready(
                    state.mode,
                    altitude,
                    vertical_velocity,
                    self.config["takeoff_height"],
                    self.args.takeoff_tolerance,
                    self.args.takeoff_max_vertical_speed,
                )
            )
            if ready:
                if stable_since is None:
                    stable_since = now
                if now - stable_since >= self.args.takeoff_stable_time:
                    self.rospy.loginfo(
                        "PX4 native takeoff settled in %s: altitude=%.2f m, "
                        "vertical speed=%.2f m/s, stable for %.2f s.",
                        state.mode,
                        altitude,
                        vertical_velocity,
                        self.args.takeoff_stable_time,
                    )
                    return
            else:
                stable_since = None
            with self.condition:
                self.condition.wait(0.05)
        raise FlightDirectorError(
            "PX4 native takeoff did not settle within {:.2f} +/- {:.2f} m "
            "with |vertical speed| <= {:.2f} m/s within {:.1f}s".format(
                self.config["takeoff_height"],
                self.args.takeoff_tolerance,
                self.args.takeoff_max_vertical_speed,
                self.args.takeoff_timeout,
            )
        )

    def _settle_before_offboard(self):
        settle_time = self.config["takeoff_settle_time"]
        if settle_time <= 0.0:
            return
        deadline = time.monotonic() + settle_time
        self.rospy.loginfo(
            "Holding native-takeoff hover for %.2f s before OFFBOARD.",
            settle_time,
        )
        while time.monotonic() < deadline:
            self._check_abort()
            self._check_localization_health()
            state, _ = self._state_snapshot()
            if (
                state is None
                or not self._state_is_fresh()
                or not state.connected
                or not state.armed
                or state.mode not in ("AUTO.TAKEOFF", "AUTO.LOITER")
            ):
                raise FlightDirectorError(
                    "flight state changed during takeoff settle time",
                    EXIT_MANUAL_TAKEOVER,
                )
            with self.condition:
                self.condition.wait(max(0.0, min(0.05, deadline - time.monotonic())))

    def _enter_and_verify_offboard(self):
        self._check_localization_health()
        state, _ = self._state_snapshot()
        already_offboard = state is not None and state.mode == "OFFBOARD"
        with self.condition:
            attitude_baseline = self.attitude_setpoint_count

        if not already_offboard:
            with self.condition:
                position_baseline = self.position_setpoint_count
            warmup_deadline = time.monotonic() + self.args.preflight_timeout
            while True:
                self._check_abort()
                self._check_localization_health()
                with self.condition:
                    enough_position_setpoints = (
                        self.position_setpoint_count - position_baseline >= 10
                    )
                if enough_position_setpoints:
                    break
                remaining = warmup_deadline - time.monotonic()
                if remaining <= 0.0:
                    raise FlightDirectorError(
                        "ten fresh hold setpoints were not available before "
                        "OFFBOARD"
                    )
                with self.condition:
                    self.condition.wait(min(remaining, 0.1))

            deadline = time.monotonic() + self.args.command_timeout
            while time.monotonic() < deadline:
                self._check_abort()
                self._check_localization_health()
                state, _ = self._state_snapshot()
                if (
                    state is None
                    or not self._state_is_fresh()
                    or not state.connected
                    or not state.armed
                ):
                    raise FlightDirectorError(
                        "PX4 state is not connected and armed before OFFBOARD"
                    )
                if state.mode == "OFFBOARD":
                    break
                if state.mode not in ("AUTO.TAKEOFF", "AUTO.LOITER"):
                    raise FlightDirectorError(
                        "flight mode changed to {} before OFFBOARD".format(
                            state.mode or "unknown"
                        ),
                        EXIT_MANUAL_TAKEOVER,
                    )
                try:
                    self.set_mode(base_mode=0, custom_mode="OFFBOARD")
                except self.rospy.ServiceException:
                    pass
                with self.condition:
                    self.condition.wait(0.1)
            else:
                raise FlightDirectorError(
                    "PX4 did not enter OFFBOARD within {:.1f}s".format(
                        self.args.command_timeout
                    )
                )
        else:
            self.rospy.loginfo(
                "Vehicle is already armed in OFFBOARD; skipping the obsolete "
                "position-setpoint warmup and verifying fresh SE3 output."
            )

        attitude_deadline = time.monotonic() + self.args.preflight_timeout
        while True:
            self._check_abort()
            self._check_localization_health()
            now = time.monotonic()
            with self.condition:
                enough_attitude_setpoints = (
                    self.attitude_setpoint_count - attitude_baseline >= 5
                )
                state = self.state
                state_received_at = self.state_received_at
            if (
                state is None
                or now - state_received_at > self.config["state_timeout"]
                or not state.connected
                or not state.armed
                or state.mode != "OFFBOARD"
            ):
                raise FlightDirectorError(
                    "PX4 left armed OFFBOARD before SE3 output was verified",
                    EXIT_MANUAL_TAKEOVER,
                )
            if enough_attitude_setpoints:
                break
            remaining = attitude_deadline - time.monotonic()
            if remaining <= 0.0:
                raise FlightDirectorError(
                    "five SE3 attitude/thrust setpoints were not observed "
                    "after OFFBOARD"
                )
            with self.condition:
                self.condition.wait(min(remaining, 0.1))
        self.rospy.loginfo("PX4 is armed in OFFBOARD; starting waypoints now.")

    def _prepare_flight(self):
        self.rospy.loginfo("Running shared simulation/real mission preflight.")
        self._check_localization_interlock()
        self._wait_for_services()
        self._wait_for_preflight_data()
        self._check_localization_interlock()
        state, _ = self._state_snapshot()
        if state.armed:
            if state.mode != "OFFBOARD":
                raise FlightDirectorError(
                    "vehicle is already armed in {}; treating it as pilot control".format(
                        state.mode or "unknown"
                    ),
                    EXIT_MANUAL_TAKEOVER,
                )
            self._enter_and_verify_offboard()
            return
        if disarmed_mode_requires_reset(
            state.mode, self.args.disarmed_prearm_mode
        ):
            self._reset_disarmed_mode()
        self._configure_takeoff()
        self.rospy.logwarn(
            "Requesting PX4 arming and AUTO.TAKEOFF; propellers may start immediately."
        )
        self._arm_and_start_takeoff()
        self._wait_for_native_takeoff_settle()
        self._settle_before_offboard()
        self._restore_altitude_acceptance_radius()
        self._enter_and_verify_offboard()

    def _request_land(self):
        deadline = time.monotonic() + self.args.command_timeout
        auto_land_active = False
        while time.monotonic() < deadline:
            self._check_abort()
            state, _ = self._state_snapshot()
            if (
                state is None
                or not self._state_is_fresh()
                or not state.connected
            ):
                raise FlightDirectorError("PX4 disconnected before mission landing")
            if not state.armed:
                self.rospy.loginfo("Vehicle is already disarmed after mission.")
                if disarmed_mode_requires_reset(
                    state.mode, self.args.disarmed_prearm_mode
                ):
                    self._reset_disarmed_mode()
                return
            if state.mode == "AUTO.LAND":
                self.rospy.loginfo(
                    "PX4 AUTO.LAND is active; waiting for touchdown and PX4 "
                    "automatic disarm."
                )
                auto_land_active = True
                break
            if state.mode != "OFFBOARD":
                raise FlightDirectorError(
                    "flight mode changed to {} before AUTO.LAND".format(
                        state.mode or "unknown"
                    ),
                    EXIT_MANUAL_TAKEOVER,
                )
            try:
                self.set_mode(base_mode=0, custom_mode="AUTO.LAND")
            except self.rospy.ServiceException:
                pass
            with self.condition:
                self.condition.wait(0.1)
        if not auto_land_active:
            raise FlightDirectorError(
                "PX4 did not enter AUTO.LAND within {:.1f}s".format(
                    self.args.command_timeout
                )
            )

        deadline = time.monotonic() + self.args.landing_timeout
        while time.monotonic() < deadline:
            self._check_abort()
            state, _ = self._state_snapshot()
            if (
                state is None
                or not self._state_is_fresh()
                or not state.connected
            ):
                raise FlightDirectorError("PX4 disconnected during mission landing")
            if not state.armed:
                self.rospy.loginfo(
                    "PX4 landing completed and the vehicle is disarmed."
                )
                if disarmed_mode_requires_reset(
                    state.mode, self.args.disarmed_prearm_mode
                ):
                    self._reset_disarmed_mode()
                return
            if state.mode != "AUTO.LAND":
                raise FlightDirectorError(
                    "flight mode changed to {} during AUTO.LAND".format(
                        state.mode or "unknown"
                    ),
                    EXIT_MANUAL_TAKEOVER,
                )
            with self.condition:
                self.condition.wait(0.1)
        raise FlightDirectorError(
            "PX4 remained armed in AUTO.LAND for more than {:.1f}s; inspect "
            "the vehicle and take over with the RC".format(
                self.args.landing_timeout
            )
        )

    def _request_safe_recovery(self, reason):
        """Confirm LOITER after a director failure, falling back to LAND."""
        state, _ = self._state_snapshot()
        if (
            state is None
            or not self._state_is_fresh()
            or not state.connected
            or not state.armed
        ):
            return
        if state.mode == "AUTO.LAND":
            self.rospy.logwarn(
                "PX4 AUTO.LAND is already active after mission failure."
            )
            return
        autonomous_modes = ("AUTO.TAKEOFF", "AUTO.LOITER", "OFFBOARD")
        if state.mode not in autonomous_modes:
            self.rospy.logwarn(
                "Mission recovery did not override pilot mode %s.",
                state.mode or "unknown",
            )
            return

        target = flight_director_recovery_mode(reason)
        targets = (target,) if target == "AUTO.LAND" else (target, "AUTO.LAND")
        for target_mode in targets:
            self.rospy.logerr(
                "Mission director failure: %s. Requesting and confirming %s.",
                reason,
                target_mode,
            )
            deadline = time.monotonic() + self.args.command_timeout
            while time.monotonic() < deadline:
                state, _ = self._state_snapshot()
                if (
                    state is None
                    or not self._state_is_fresh()
                    or not state.connected
                    or not state.armed
                ):
                    break
                if state.mode == target_mode:
                    self.rospy.logwarn(
                        "PX4 %s is active after mission failure.", target_mode
                    )
                    return
                if target_mode == "AUTO.LOITER" and state.mode == "AUTO.LAND":
                    self.rospy.logwarn(
                        "PX4 AUTO.LAND is already active after mission failure."
                    )
                    return
                if state.mode not in autonomous_modes:
                    self.rospy.logwarn(
                        "Mission recovery stopped after mode changed to %s; "
                        "not overriding the pilot.",
                        state.mode or "unknown",
                    )
                    return
                try:
                    response = self.set_mode(
                        base_mode=0, custom_mode=target_mode
                    )
                    if not response.mode_sent:
                        self.rospy.logerr_throttle(
                            1.0,
                            "PX4 rejected %s during mission recovery.",
                            target_mode,
                        )
                except Exception as exc:
                    self.rospy.logerr_throttle(
                        1.0,
                        "Unable to request %s during mission recovery (%s).",
                        target_mode,
                        exc,
                    )
                with self.condition:
                    self.condition.wait(0.1)
            if target_mode == "AUTO.LOITER":
                self.rospy.logerr(
                    "AUTO.LOITER could not be confirmed; falling back to "
                    "AUTO.LAND."
                )
        self.rospy.logerr(
            "No safe PX4 recovery mode could be confirmed; take over "
            "immediately with the RC."
        )

    def run(self):
        try:
            valid, reason = self.runner.validate_all_goals()
            if not valid:
                raise FlightDirectorError(reason)
            self._prepare_flight()
            result = self.runner.run()
            if result != EXIT_SUCCESS:
                return result
            if self.config["land_after_mission"]:
                self._request_land()
            else:
                self.rospy.loginfo(
                    "All waypoints reached; holding the final point in OFFBOARD."
                )
            return EXIT_SUCCESS
        except FlightDirectorError as exc:
            if exc.code == EXIT_MANUAL_TAKEOVER:
                self.rospy.logwarn(
                    "Mission stopped for pilot/manual takeover: %s", exc
                )
            else:
                self.rospy.logerr("Mission flight director failed: %s", exc)
                self._request_safe_recovery(str(exc))
            return exc.code
        except Exception as exc:
            self.rospy.logerr("Unexpected mission executor failure: %s", exc)
            self._request_safe_recovery(
                "unexpected mission executor error: {}".format(exc)
            )
            return EXIT_MISSION_FAILED
        finally:
            self._restore_altitude_acceptance_radius()


def _build_parser():
    parser = argparse.ArgumentParser(
        description="Execute the shared simulation/real PX4 waypoint mission"
    )
    parser.add_argument("mission_file")
    parser.add_argument("--drone-id", type=int, default=0)
    parser.add_argument("--default-takeoff-height", type=float, default=1.0)
    parser.add_argument(
        "--px4-hover-thrust",
        type=float,
        default=None,
        help=(
            "Optional MPC_THR_HOVER calibration used by PX4 native takeoff. "
            "Omit on real flight to retain the autopilot's stored value."
        ),
    )
    parser.add_argument(
        "--disarmed-prearm-mode",
        choices=("STABILIZED", "AUTO.LOITER"),
        default="STABILIZED",
        help=(
            "Safe mode selected before arming. Simulation without RC should "
            "use AUTO.LOITER; real flight defaults to STABILIZED."
        ),
    )
    parser.add_argument("--preflight-timeout", type=float, default=5.0)
    parser.add_argument("--command-timeout", type=float, default=15.0)
    parser.add_argument("--takeoff-timeout", type=float, default=30.0)
    parser.add_argument("--landing-timeout", type=float, default=120.0)
    parser.add_argument("--takeoff-tolerance", type=float, default=0.1)
    parser.add_argument("--takeoff-stable-time", type=float, default=0.5)
    parser.add_argument(
        "--takeoff-max-vertical-speed", type=float, default=0.2
    )
    parser.add_argument(
        "--odom-timeout",
        type=float,
        default=None,
        help=(
            "Optional override for the mission JSON odom_timeout; the same "
            "value is used by preflight and the waypoint runner."
        ),
    )
    parser.add_argument("--altitude-timeout", type=float, default=0.5)
    parser.add_argument("--state-topic", default="/mavros/state")
    parser.add_argument("--altitude-topic", default="/mavros/altitude")
    parser.add_argument("--odometry-topic", default="/localization/odom")
    return parser


def main(argv=None):
    args = _build_parser().parse_args(argv)
    for name in (
        "default_takeoff_height",
        "preflight_timeout",
        "command_timeout",
        "takeoff_timeout",
        "landing_timeout",
        "takeoff_tolerance",
        "takeoff_stable_time",
        "takeoff_max_vertical_speed",
        "altitude_timeout",
    ):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0.0:
            print("[ERROR] {} must be finite and positive".format(name), file=sys.stderr)
            return EXIT_MISSION_FAILED
    if args.odom_timeout is not None and (
        not math.isfinite(args.odom_timeout) or args.odom_timeout <= 0.0
    ):
        print("[ERROR] odom_timeout must be finite and positive", file=sys.stderr)
        return EXIT_MISSION_FAILED
    if args.px4_hover_thrust is not None and (
        not math.isfinite(args.px4_hover_thrust)
        or args.px4_hover_thrust <= 0.0
        or args.px4_hover_thrust > 1.0
    ):
        print(
            "[ERROR] px4_hover_thrust must be within (0, 1]",
            file=sys.stderr,
        )
        return EXIT_MISSION_FAILED

    try:
        import rospy

        rospy.init_node("shared_waypoint_mission", disable_signals=True)
        config = load_mission_config(
            args.mission_file,
            default_takeoff_height=args.default_takeoff_height,
        )
        if args.odom_timeout is not None:
            config["odom_timeout"] = args.odom_timeout
    except (ImportError, KeyError, ValueError, MissionConfigError) as exc:
        print("[ERROR] Cannot initialize mission: {}".format(exc), file=sys.stderr)
        return EXIT_MISSION_FAILED

    if args.takeoff_tolerance >= config["takeoff_height"]:
        print(
            "[ERROR] takeoff_tolerance must be below mission takeoff_height",
            file=sys.stderr,
        )
        return EXIT_MISSION_FAILED

    try:
        executor = SharedMissionExecutor(rospy, config, args)
    except Exception as exc:
        print(
            "[ERROR] Cannot construct mission executor: {}".format(exc),
            file=sys.stderr,
        )
        return EXIT_MISSION_FAILED
    previous_sigint = signal.getsignal(signal.SIGINT)
    previous_sigterm = signal.getsignal(signal.SIGTERM)

    def request_abort(_signum, _frame):
        executor.request_abort()

    signal.signal(signal.SIGINT, request_abort)
    signal.signal(signal.SIGTERM, request_abort)
    try:
        return executor.run()
    finally:
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)
        rospy.signal_shutdown("shared mission finished")


if __name__ == "__main__":
    sys.exit(main())
