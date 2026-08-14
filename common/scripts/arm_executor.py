#!/usr/bin/env python3
"""Shared low-latency PX4 arm/takeoff/OFFBOARD lifecycle.

Both launchers stream this exact file into their existing ROS container.  A
single process keeps all subscriptions and service proxies alive, avoiding the
ROS discovery and Docker startup delay of repeated ``rostopic``/``rosservice``
commands while preserving the same safety checks in simulation and real flight.
"""

import argparse
import math
import signal
import sys
import threading
import time


EXIT_SUCCESS = 0
EXIT_FAILED = 1
EXIT_MANUAL_TAKEOVER = 10
MAV_STATE_FLIGHT_TERMINATION = 8
AUTOMATIC_TAKEOFF_MODES = ("AUTO.TAKEOFF", "AUTO.LOITER")
LOCALIZATION_FAULT_PARAM = "/sim2real/localization_fault"


class ArmExecutorError(RuntimeError):
    def __init__(self, message, code=EXIT_FAILED):
        super().__init__(message)
        self.code = code


def native_takeoff_target(height, acceptance_radius):
    values = (height, acceptance_radius)
    if not all(math.isfinite(value) for value in values):
        raise ArmExecutorError(
            "takeoff height and NAV_MC_ALT_RAD must be finite"
        )
    if height <= 0.0 or acceptance_radius < 0.0:
        raise ArmExecutorError(
            "takeoff height must be positive and NAV_MC_ALT_RAD non-negative"
        )
    # Flight evidence from the real PX4 shows MIS_TAKEOFF_ALT is the actual
    # altitude setpoint. NAV_MC_ALT_RAD does not reduce that setpoint or cause
    # this externally requested AUTO.TAKEOFF to leave the mode. Adding the
    # acceptance radius therefore makes a 1.0 m request climb to 1.8 m.
    return height


def temporary_takeoff_acceptance_radius(current_radius, altitude_tolerance):
    values = (current_radius, altitude_tolerance)
    if not all(math.isfinite(value) and value > 0.0 for value in values):
        raise ArmExecutorError(
            "takeoff acceptance radius inputs must be finite and positive"
        )
    return min(current_radius, altitude_tolerance)


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


def localization_fault_reason(value):
    """Return the persistent interlock reason, or empty when it is clear."""
    if isinstance(value, dict):
        if not value.get("active", False):
            return ""
        return str(value.get("reason") or "localization safety fault")
    if value:
        return str(value)
    return ""


def arm_failure_recovery_modes(reason):
    reason = str(reason).lower()
    if any(token in reason for token in ("localization", "odometry", "odom")):
        return ("AUTO.LAND",)
    return ("AUTO.LOITER", "AUTO.LAND")


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
        mode in AUTOMATIC_TAKEOFF_MODES
        and float(height) - float(tolerance)
        <= float(altitude)
        <= float(height) + float(tolerance)
        and abs(float(vertical_speed)) <= float(max_vertical_speed)
    )


def select_takeoff_altitude(field, local, relative, target):
    """Select the MAVROS altitude that represents the PX4 takeoff target."""
    candidates = {"local": local, "relative": relative}
    if field != "auto":
        return field, candidates[field]

    finite = [
        (name, float(value))
        for name, value in candidates.items()
        if value is not None and math.isfinite(float(value))
    ]
    if not finite:
        return "auto", None
    return min(finite, key=lambda item: abs(item[1] - float(target)))


class SharedArmExecutor:
    """One arm state machine used unchanged by SITL and real flight."""

    def __init__(self, rospy, args):
        import rosnode
        from geometry_msgs.msg import PoseStamped
        from mavros_msgs.msg import Altitude, AttitudeTarget, ParamValue, State
        from mavros_msgs.srv import CommandBool, ParamGet, ParamSet, SetMode
        from nav_msgs.msg import Odometry

        self.rospy = rospy
        self.rosnode = rosnode
        self.ParamValue = ParamValue
        self.args = args
        self.condition = threading.Condition()
        self.abort_requested = False
        self.state = None
        self.state_received_at = 0.0
        self.takeoff_altitude = None
        self.altitude_received_at = 0.0
        self.takeoff_altitude_source = args.takeoff_altitude_field
        self.vertical_velocity = None
        self.odom_received_at = 0.0
        self.position_setpoint_count = 0
        self.position_setpoint_received_at = 0.0
        self.preflight_position_setpoint_count = 0
        self.attitude_setpoint_count = 0
        self.attitude_setpoint_received_at = 0.0
        self.original_altitude_acceptance_radius = None
        self.altitude_acceptance_radius_changed = False
        self.started_at = time.monotonic()

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
            args.position_setpoint_topic,
            PoseStamped,
            self._position_setpoint_callback,
            queue_size=20,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            args.attitude_setpoint_topic,
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
            (
                self.takeoff_altitude_source,
                self.takeoff_altitude,
            ) = select_takeoff_altitude(
                self.args.takeoff_altitude_field,
                message.local,
                message.relative,
                self.args.takeoff_height,
            )
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
            self.attitude_setpoint_received_at = time.monotonic()
            self.condition.notify_all()

    def request_abort(self):
        with self.condition:
            self.abort_requested = True
            self.condition.notify_all()

    def _check_abort(self):
        if self.abort_requested or self.rospy.is_shutdown():
            raise ArmExecutorError("arm executor was interrupted")

    def _check_localization_interlock(self):
        try:
            value = self.rospy.get_param(LOCALIZATION_FAULT_PARAM, "")
        except Exception as exc:
            raise ArmExecutorError(
                "localization safety interlock could not be read: {}".format(
                    exc
                )
            )
        reason = localization_fault_reason(value)
        if reason:
            raise ArmExecutorError(
                "localization safety interlock is latched: {}. Restart the "
                "complete simulation/real stack before arming again".format(
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
        if odom_received_at <= 0.0 or age > self.args.odom_timeout:
            raise ArmExecutorError(
                "localization odometry is unavailable or stale "
                "(age {:.2f}s, limit {:.2f}s)".format(
                    age, self.args.odom_timeout
                )
            )

    def _state_snapshot(self):
        with self.condition:
            return self.state, self.takeoff_altitude

    def _state_is_fresh(self, now=None):
        if now is None:
            now = time.monotonic()
        with self.condition:
            return (
                self.state is not None
                and now - self.state_received_at <= self.args.state_timeout
            )

    def _altitude_is_fresh(self, now=None):
        if now is None:
            now = time.monotonic()
        with self.condition:
            return (
                self.takeoff_altitude is not None
                and now - self.altitude_received_at
                <= self.args.altitude_timeout
            )

    def _wait_for_services(self):
        deadline = time.monotonic() + self.args.preflight_timeout
        for name in (
            "/mavros/cmd/arming",
            "/mavros/set_mode",
            "/mavros/param/get",
            "/mavros/param/set",
        ):
            self._check_abort()
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise ArmExecutorError("required MAVROS services are unavailable")
            try:
                self.rospy.wait_for_service(name, timeout=remaining)
            except self.rospy.ROSException as exc:
                raise ArmExecutorError(
                    "required service {} is unavailable: {}".format(name, exc)
                )

    def _wait_for_preflight_data(self):
        try:
            nodes = self.rosnode.get_node_names()
        except Exception as exc:
            raise ArmExecutorError("cannot query ROS nodes: {}".format(exc))
        if self.args.controller_node not in nodes:
            raise ArmExecutorError(
                "controller node is not running: {}".format(
                    self.args.controller_node
                )
            )

        deadline = time.monotonic() + self.args.preflight_timeout
        while True:
            self._check_abort()
            self._check_localization_interlock()
            now = time.monotonic()
            with self.condition:
                state = self.state
                state_received_at = self.state_received_at
                takeoff_altitude = self.takeoff_altitude
                altitude_received_at = self.altitude_received_at
                odom_received_at = self.odom_received_at
                position_setpoint_count = self.position_setpoint_count
                position_setpoint_received_at = (
                    self.position_setpoint_received_at
                )
            position_stream_ready = (
                position_setpoint_count
                >= self.args.position_setpoint_samples
                and now - position_setpoint_received_at
                <= self.args.odom_timeout
            )
            state_is_fresh = (
                state is not None
                and now - state_received_at <= self.args.state_timeout
            )
            altitude_is_fresh = (
                takeoff_altitude is not None
                and now - altitude_received_at <= self.args.altitude_timeout
            )
            odom_is_fresh = (
                odom_received_at > 0.0
                and now - odom_received_at <= self.args.odom_timeout
            )
            if state_is_fresh and state.armed and not odom_is_fresh:
                raise ArmExecutorError(
                    "localization odometry is unavailable or stale while "
                    "the vehicle is armed"
                )
            ready = (
                state_is_fresh
                and altitude_is_fresh
                and math.isfinite(float(takeoff_altitude))
                and odom_is_fresh
                # Once armed, manual takeover and already-OFFBOARD handling
                # must not depend on the pre-OFFBOARD position stream. The
                # latter is replaced by attitude/thrust output in OFFBOARD.
                and (state.armed or position_stream_ready)
            )
            if ready:
                with self.condition:
                    self.preflight_position_setpoint_count = (
                        self.position_setpoint_count
                    )
                break
            remaining = deadline - now
            if remaining <= 0.0:
                raise ArmExecutorError(
                    "fresh MAVROS state, altitude, localization and {} "
                    "continuous hold setpoints were not ready within "
                    "{:.1f}s".format(
                        self.args.position_setpoint_samples,
                        self.args.preflight_timeout,
                    )
                )
            with self.condition:
                self.condition.wait(min(remaining, 0.1))

        if not state.connected:
            raise ArmExecutorError("MAVROS is not connected to PX4")
        if (
            not state.armed
            and px4_flight_termination_active(state.system_status)
        ):
            raise ArmExecutorError(
                "PX4 reports FLIGHT_TERMINATION (system_status=8); release "
                "the RC kill switch and clear the termination state before "
                "arming"
            )
        self.rospy.loginfo(
            "Shared arm preflight ready after %.2f s.",
            time.monotonic() - self.started_at,
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
                raise ArmExecutorError("PX4 state became unavailable")
            if state.armed:
                raise ArmExecutorError(
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
        raise ArmExecutorError(
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
                raise ArmExecutorError(
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
                raise ArmExecutorError(
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
            raise ArmExecutorError(
                "unable to read NAV_MC_ALT_RAD: {}".format(exc)
            )
        if not radius_response.success:
            raise ArmExecutorError("PX4 rejected the NAV_MC_ALT_RAD read")

        acceptance_radius = float(radius_response.value.real)
        temporary_radius = temporary_takeoff_acceptance_radius(
            acceptance_radius, self.args.takeoff_tolerance
        )
        target = native_takeoff_target(
            self.args.takeoff_height, acceptance_radius
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
            raise ArmExecutorError(
                "unable to configure temporary NAV_MC_ALT_RAD: {}".format(exc)
            )
        if (
            not radius_set_response.success
            or abs(float(radius_set_response.value.real) - temporary_radius)
            > 1e-3
        ):
            raise ArmExecutorError(
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
            raise ArmExecutorError(
                "unable to configure PX4 native takeoff: {}".format(exc)
            )
        if (
            not altitude_response.success
            or abs(float(altitude_response.value.real) - target) > 1e-3
        ):
            raise ArmExecutorError(
                "PX4 rejected MIS_TAKEOFF_ALT={}".format(target)
            )
        if not action_response.success or action_response.value.integer != 0:
            raise ArmExecutorError("PX4 rejected COM_TAKEOFF_ACT=0")

        self.rospy.loginfo(
            "Native takeoff ready: desired=%.2f m, temporary "
            "NAV_MC_ALT_RAD=%.2f m (original %.2f m), "
            "MIS_TAKEOFF_ALT=%.2f m.",
            self.args.takeoff_height,
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
        arm_accepted = False
        while time.monotonic() < deadline:
            self._check_abort()
            self._check_localization_health()
            try:
                response = self.arm_vehicle(value=True)
            except self.rospy.ServiceException:
                response = None
            if response is not None and response.success:
                arm_accepted = True
                break
            with self.condition:
                self.condition.wait(0.1)
        if not arm_accepted:
            raise ArmExecutorError(
                "PX4 did not arm within {:.1f}s; check PX4 preflight messages, "
                "the RC kill switch and QGC".format(self.args.command_timeout)
            )

        arm_elapsed = time.monotonic() - self.started_at
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
                raise ArmExecutorError("PX4 disconnected before AUTO.TAKEOFF")
            if state.armed and state.mode == "AUTO.TAKEOFF":
                self.rospy.loginfo(
                    "PX4 armed and AUTO.TAKEOFF active after %.2f s.",
                    time.monotonic() - self.started_at,
                )
                return
            try:
                self.set_mode(base_mode=0, custom_mode="AUTO.TAKEOFF")
            except self.rospy.ServiceException:
                pass
            with self.condition:
                self.condition.wait(0.1)
        raise ArmExecutorError(
            "PX4 armed after {:.2f}s but did not enter AUTO.TAKEOFF within "
            "{:.1f}s".format(arm_elapsed, self.args.command_timeout)
        )

    def _wait_for_native_takeoff_settle(self):
        started = time.monotonic()
        deadline = started + self.args.takeoff_timeout
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
                raise ArmExecutorError(
                    "PX4 state became stale, disconnected or disarmed during "
                    "AUTO.TAKEOFF"
                )
            if state.mode not in AUTOMATIC_TAKEOFF_MODES:
                raise ArmExecutorError(
                    "flight mode changed to {} during takeoff".format(
                        state.mode or "unknown"
                    ),
                    EXIT_MANUAL_TAKEOVER,
                )
            if (
                altitude is not None
                and math.isfinite(float(altitude))
                and altitude
                >= self.args.takeoff_height - self.args.takeoff_tolerance
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
                altitude_source = self.takeoff_altitude_source
                odom_is_fresh = (
                    now - self.odom_received_at <= self.args.odom_timeout
                )
                altitude_is_fresh = (
                    self.takeoff_altitude is not None
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
                    self.args.takeoff_height,
                    self.args.takeoff_tolerance,
                    self.args.takeoff_max_vertical_speed,
                )
            )
            if ready:
                if stable_since is None:
                    stable_since = now
                if now - stable_since >= self.args.takeoff_stable_time:
                    self.rospy.loginfo(
                        "PX4 native takeoff settled in %s: %s altitude=%.2f m, "
                        "vertical speed=%.2f m/s, stable for %.2f s "
                        "(%.2f s total).",
                        state.mode,
                        altitude_source,
                        altitude,
                        vertical_velocity,
                        self.args.takeoff_stable_time,
                        now - self.started_at,
                    )
                    return now
            else:
                stable_since = None
            with self.condition:
                self.condition.wait(0.05)
        raise ArmExecutorError(
            "PX4 native takeoff did not settle within {:.2f} +/- {:.2f} m "
            "with |vertical speed| <= {:.2f} m/s within {:.1f}s".format(
                self.args.takeoff_height,
                self.args.takeoff_tolerance,
                self.args.takeoff_max_vertical_speed,
                self.args.takeoff_timeout,
            )
        )

    def _wait_for_fresh_hold_setpoints(self):
        deadline = time.monotonic() + self.args.preflight_timeout
        while True:
            self._check_abort()
            self._check_localization_health()
            now = time.monotonic()
            with self.condition:
                state = self.state
                state_received_at = self.state_received_at
                enough_during_takeoff = (
                    self.position_setpoint_count
                    - self.preflight_position_setpoint_count
                    >= self.args.position_setpoint_samples
                )
                stream_is_fresh = (
                    now - self.position_setpoint_received_at
                    <= self.args.odom_timeout
                )
            if (
                state is None
                or now - state_received_at > self.args.state_timeout
                or not state.connected
                or not state.armed
                or state.mode not in AUTOMATIC_TAKEOFF_MODES
            ):
                mode = state.mode if state is not None else "unknown"
                raise ArmExecutorError(
                    "flight mode changed to {} before OFFBOARD".format(mode),
                    EXIT_MANUAL_TAKEOVER,
                )
            if enough_during_takeoff and stream_is_fresh:
                return
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise ArmExecutorError(
                    "{} sustained hold setpoints were not observed during "
                    "takeoff before OFFBOARD".format(
                        self.args.position_setpoint_samples
                    )
                )
            with self.condition:
                self.condition.wait(min(remaining, 0.1))

    def _enter_and_verify_offboard(self, takeoff_settled_at=None):
        self._check_localization_health()
        state, _ = self._state_snapshot()
        already_offboard = state is not None and state.mode == "OFFBOARD"
        with self.condition:
            attitude_baseline = self.attitude_setpoint_count
            position_baseline = self.position_setpoint_count

        if not already_offboard:
            self._wait_for_fresh_hold_setpoints()
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
                    raise ArmExecutorError(
                        "PX4 state is not connected and armed before OFFBOARD"
                    )
                if state.mode == "OFFBOARD":
                    # Only setpoints published after PX4 confirms OFFBOARD can
                    # prove that the controller kept its local-position hold
                    # alive through the mode transition.
                    with self.condition:
                        position_baseline = self.position_setpoint_count
                    break
                if state.mode not in AUTOMATIC_TAKEOFF_MODES:
                    raise ArmExecutorError(
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
                raise ArmExecutorError(
                    "PX4 did not enter OFFBOARD within {:.1f}s".format(
                        self.args.command_timeout
                    )
                )

        control_deadline = time.monotonic() + self.args.preflight_timeout
        while True:
            self._check_abort()
            self._check_localization_health()
            now = time.monotonic()
            with self.condition:
                enough_position_setpoints = (
                    self.position_setpoint_count - position_baseline
                    >= self.args.position_setpoint_samples
                    and now - self.position_setpoint_received_at
                    <= self.args.odom_timeout
                )
                enough_attitude_setpoints = (
                    self.attitude_setpoint_count - attitude_baseline
                    >= self.args.attitude_setpoint_samples
                    and now - self.attitude_setpoint_received_at
                    <= self.args.odom_timeout
                )
                state = self.state
                state_received_at = self.state_received_at
            if (
                state is None
                or now - state_received_at > self.args.state_timeout
                or not state.connected
                or not state.armed
                or state.mode != "OFFBOARD"
            ):
                raise ArmExecutorError(
                    "PX4 left armed OFFBOARD before its control stream was verified",
                    EXIT_MANUAL_TAKEOVER,
                )
            # A newly completed native takeoff must enter OFFBOARD using the
            # PX4 local-position hold stream.  Raw SE3 attitude output is
            # intentionally deferred until a planner trajectory exists.  If
            # this process was invoked while already in OFFBOARD, accept either
            # the position-hold stream or an already-running SE3 trajectory.
            control_stream_ready = enough_position_setpoints or (
                already_offboard and enough_attitude_setpoints
            )
            if control_stream_ready:
                break
            remaining = control_deadline - time.monotonic()
            if remaining <= 0.0:
                expected = (
                    "PX4 local-position hold"
                    if not already_offboard
                    else "PX4 local-position hold or SE3 attitude/thrust"
                )
                raise ArmExecutorError(
                    "sustained {} setpoints were not observed after "
                    "OFFBOARD".format(
                        expected
                    )
                )
            with self.condition:
                self.condition.wait(min(remaining, 0.1))

        now = time.monotonic()
        if takeoff_settled_at is None:
            self.rospy.loginfo(
                "Vehicle was already armed in OFFBOARD; a fresh position or "
                "attitude control stream was verified in %.2f s.",
                now - self.started_at,
            )
        else:
            self.rospy.loginfo(
                "OFFBOARD PX4 local-position hold verified %.2f s after native "
                "takeoff settled (%.2f s total).",
                now - takeoff_settled_at,
                now - self.started_at,
            )

    def _request_hold_once(self, reason):
        state, _ = self._state_snapshot()
        if (
            state is None
            or not self._state_is_fresh()
            or not state.connected
            or not state.armed
        ):
            return
        recovery_modes = arm_failure_recovery_modes(reason)
        localization_unsafe = recovery_modes == ("AUTO.LAND",)
        if state.mode == "AUTO.LOITER" and not localization_unsafe:
            self.rospy.logwarn("PX4 is already holding in AUTO.LOITER.")
            return
        if state.mode not in ("AUTO.TAKEOFF", "AUTO.LOITER", "OFFBOARD"):
            self.rospy.logwarn(
                "Not overriding current mode %s after failure: %s",
                state.mode or "unknown",
                reason,
            )
            return
        for target_mode in recovery_modes:
            deadline = time.monotonic() + self.args.command_timeout
            while time.monotonic() < deadline:
                state, _ = self._state_snapshot()
                if (
                    state is None
                    or not self._state_is_fresh()
                    or not state.connected
                    or not state.armed
                ):
                    return
                if state.mode == target_mode:
                    self.rospy.logwarn(
                        "PX4 %s is confirmed after arm failure.", target_mode
                    )
                    return
                if target_mode == "AUTO.LOITER" and state.mode == "AUTO.LAND":
                    self.rospy.logwarn(
                        "PX4 AUTO.LAND is already active after arm failure."
                    )
                    return
                if state.mode not in ("AUTO.TAKEOFF", "AUTO.LOITER", "OFFBOARD"):
                    self.rospy.logwarn(
                        "Not overriding current mode %s after failure: %s",
                        state.mode or "unknown",
                        reason,
                    )
                    return
                try:
                    response = self.set_mode(
                        base_mode=0, custom_mode=target_mode
                    )
                    if not response.mode_sent:
                        self.rospy.logerr_throttle(
                            1.0,
                            "PX4 rejected %s after arm failure.",
                            target_mode,
                        )
                except Exception as exc:
                    self.rospy.logerr_throttle(
                        1.0,
                        "Failed to request %s after arm failure: %s",
                        target_mode,
                        exc,
                    )
                with self.condition:
                    self.condition.wait(0.1)
            if target_mode == "AUTO.LOITER":
                self.rospy.logerr(
                    "AUTO.LOITER was not confirmed after arm failure; "
                    "falling back to AUTO.LAND."
                )
        self.rospy.logerr(
            "No safe PX4 recovery mode was confirmed after arm failure; "
            "take over immediately with the RC."
        )

    def run(self):
        try:
            self.rospy.loginfo(
                "Running the shared simulation/real arm state machine."
            )
            self._check_localization_interlock()
            self._wait_for_services()
            self._wait_for_preflight_data()
            self._check_localization_interlock()
            state, _ = self._state_snapshot()
            if state.armed:
                if state.mode != "OFFBOARD":
                    raise ArmExecutorError(
                        "vehicle is already armed in {}; treating it as pilot "
                        "control".format(state.mode or "unknown"),
                        EXIT_MANUAL_TAKEOVER,
                    )
                self._enter_and_verify_offboard()
                return EXIT_SUCCESS

            if disarmed_mode_requires_reset(
                state.mode, self.args.disarmed_prearm_mode
            ):
                self._reset_disarmed_mode()
            self._configure_takeoff()
            self.rospy.logwarn(
                "Requesting PX4 arming and AUTO.TAKEOFF; propellers may start "
                "immediately."
            )
            self._arm_and_start_takeoff()
            takeoff_settled_at = self._wait_for_native_takeoff_settle()
            self._restore_altitude_acceptance_radius()
            self._enter_and_verify_offboard(takeoff_settled_at)
            self.rospy.loginfo(
                "Arm command complete: vehicle is hovering in armed OFFBOARD "
                "using PX4 local-position hold."
            )
            return EXIT_SUCCESS
        except ArmExecutorError as exc:
            if exc.code == EXIT_MANUAL_TAKEOVER:
                self.rospy.logwarn(
                    "Arm command stopped for pilot/manual takeover: %s", exc
                )
            else:
                self.rospy.logerr("Shared arm executor failed: %s", exc)
                self._request_hold_once(str(exc))
            return exc.code
        except Exception as exc:
            self.rospy.logerr("Unexpected shared arm executor failure: %s", exc)
            self._request_hold_once(
                "unexpected arm executor error: {}".format(exc)
            )
            return EXIT_FAILED
        finally:
            self._restore_altitude_acceptance_radius()


def _build_parser():
    parser = argparse.ArgumentParser(
        description="Arm, perform PX4 native takeoff and enter verified OFFBOARD"
    )
    parser.add_argument("--takeoff-height", type=float, default=1.0)
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
        "--takeoff-altitude-field",
        choices=("relative", "local", "auto"),
        default="relative",
        help=(
            "mavros_msgs/Altitude field used for takeoff completion. "
            "Simulation can use auto to select whichever of local/relative "
            "matches the PX4 target; real flight defaults to relative."
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
    parser.add_argument("--takeoff-tolerance", type=float, default=0.1)
    parser.add_argument("--takeoff-stable-time", type=float, default=0.5)
    parser.add_argument(
        "--takeoff-max-vertical-speed", type=float, default=0.2
    )
    parser.add_argument("--odom-timeout", type=float, default=0.5)
    parser.add_argument("--state-timeout", type=float, default=3.0)
    parser.add_argument("--altitude-timeout", type=float, default=0.5)
    parser.add_argument("--state-topic", default="/mavros/state")
    parser.add_argument("--altitude-topic", default="/mavros/altitude")
    parser.add_argument("--odometry-topic", default="/localization/odom")
    parser.add_argument(
        "--position-setpoint-topic",
        default="/mavros/setpoint_position/local",
    )
    parser.add_argument(
        "--attitude-setpoint-topic",
        default="/mavros/setpoint_raw/attitude",
    )
    parser.add_argument("--controller-node", default="/se3_controller_node")
    parser.add_argument("--position-setpoint-samples", type=int, default=10)
    parser.add_argument("--attitude-setpoint-samples", type=int, default=5)
    return parser


def _validate_args(parser, args):
    for name in (
        "takeoff_height",
        "preflight_timeout",
        "command_timeout",
        "takeoff_timeout",
        "takeoff_tolerance",
        "takeoff_stable_time",
        "takeoff_max_vertical_speed",
        "odom_timeout",
        "state_timeout",
        "altitude_timeout",
    ):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0.0:
            parser.error("--{} must be finite and positive".format(name.replace("_", "-")))
    if args.takeoff_tolerance >= args.takeoff_height:
        parser.error("--takeoff-tolerance must be below --takeoff-height")
    if args.px4_hover_thrust is not None and (
        not math.isfinite(args.px4_hover_thrust)
        or args.px4_hover_thrust <= 0.0
        or args.px4_hover_thrust > 1.0
    ):
        parser.error("--px4-hover-thrust must be within (0, 1]")
    if args.position_setpoint_samples <= 0:
        parser.error("--position-setpoint-samples must be positive")
    if args.attitude_setpoint_samples <= 0:
        parser.error("--attitude-setpoint-samples must be positive")


def main(argv=None):
    parser = _build_parser()
    args = parser.parse_args(argv)
    _validate_args(parser, args)

    try:
        import rospy

        rospy.init_node("shared_arm_executor", disable_signals=True)
        executor = SharedArmExecutor(rospy, args)
    except Exception as exc:
        print("[ERROR] Cannot initialize shared arm executor: {}".format(exc), file=sys.stderr)
        return EXIT_FAILED

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
        rospy.signal_shutdown("shared arm command finished")


if __name__ == "__main__":
    sys.exit(main())
