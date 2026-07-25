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


def px4_flight_termination_active(system_status):
    return int(system_status) == MAV_STATE_FLIGHT_TERMINATION


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
        self.relative_altitude = None
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
        self.runner = WaypointMission(config, args.drone_id)

        rospy.Subscriber(
            "/mavros/state", State, self._state_callback, queue_size=1
        )
        rospy.Subscriber(
            "/mavros/altitude", Altitude, self._altitude_callback, queue_size=1
        )
        rospy.Subscriber(
            "/localization/odom",
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
            self.condition.notify_all()

    def _altitude_callback(self, message):
        with self.condition:
            self.relative_altitude = message.relative
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
        reason = localization_fault_reason(
            self.rospy.get_param(LOCALIZATION_FAULT_PARAM, "")
        )
        if reason:
            raise FlightDirectorError(
                "localization safety interlock is latched: {}. Restart the "
                "complete simulation/real stack before another mission".format(
                    reason
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
        with self.condition:
            while True:
                self._check_abort()
                self._check_localization_interlock()
                now = time.monotonic()
                setpoint_age = now - self.position_setpoint_received_at
                position_stream_ready = preflight_position_stream_ready(
                    self.state.armed if self.state is not None else False,
                    self.position_setpoint_count,
                    setpoint_age,
                    self.args.odom_timeout,
                )
                ready = (
                    self.state is not None
                    and self.relative_altitude is not None
                    and math.isfinite(float(self.relative_altitude))
                    and now - self.odom_received_at <= self.args.odom_timeout
                    and position_stream_ready
                )
                if ready:
                    break
                remaining = deadline - now
                if remaining <= 0.0:
                    missing = []
                    if self.state is None:
                        missing.append("MAVROS state")
                    if self.relative_altitude is None or not math.isfinite(
                        float(self.relative_altitude)
                    ):
                        missing.append("relative altitude")
                    if now - self.odom_received_at > self.args.odom_timeout:
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
                self.condition.wait(min(remaining, 0.1))
        if not self.state.connected:
            raise FlightDirectorError("MAVROS is not connected to PX4")
        if (
            not self.state.armed
            and px4_flight_termination_active(self.state.system_status)
        ):
            raise FlightDirectorError(
                "PX4 reports FLIGHT_TERMINATION (system_status=8); release "
                "the RC kill switch and clear the termination state before "
                "running a mission"
            )

    def _state_snapshot(self):
        with self.condition:
            return self.state, self.relative_altitude

    def _reset_disarmed_to_stabilized(self):
        self.rospy.logwarn(
            "PX4 is disarmed in %s; returning to STABILIZED before takeoff.",
            self.state.mode,
        )
        deadline = time.monotonic() + self.args.command_timeout
        while time.monotonic() < deadline:
            self._check_abort()
            state, _ = self._state_snapshot()
            if state is None or not state.connected:
                raise FlightDirectorError("PX4 state became unavailable")
            if state.armed:
                raise FlightDirectorError(
                    "PX4 armed unexpectedly while resetting its disarmed mode"
                )
            if state.mode == "STABILIZED":
                return
            if not disarmed_mode_requires_stabilized(state.mode):
                self.rospy.loginfo(
                    "PX4 is already disarmed in pilot mode %s; leaving that "
                    "mode unchanged.",
                    state.mode or "unknown",
                )
                return
            try:
                self.set_mode(base_mode=0, custom_mode="STABILIZED")
            except self.rospy.ServiceException:
                pass
            with self.condition:
                self.condition.wait(0.1)
        raise FlightDirectorError(
            "PX4 did not enter disarmed STABILIZED within {:.1f}s".format(
                self.args.command_timeout
            )
        )

    def _configure_takeoff(self):
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
        except self.rospy.ServiceException as exc:
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
            state, _ = self._state_snapshot()
            if state is not None and state.armed and state.mode == "AUTO.TAKEOFF":
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
            state, altitude = self._state_snapshot()
            if state is None or not state.connected or not state.armed:
                raise FlightDirectorError(
                    "PX4 disconnected or disarmed during AUTO.TAKEOFF"
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
                    now - self.odom_received_at <= self.args.odom_timeout
                )
            ready = (
                odom_is_fresh
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
            state, _ = self._state_snapshot()
            if (
                state is None
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
        state, _ = self._state_snapshot()
        already_offboard = state is not None and state.mode == "OFFBOARD"
        with self.condition:
            attitude_baseline = self.attitude_setpoint_count

        if not already_offboard:
            with self.condition:
                position_baseline = self.position_setpoint_count
            warmup_deadline = time.monotonic() + self.args.preflight_timeout
            with self.condition:
                while self.position_setpoint_count - position_baseline < 10:
                    self._check_abort()
                    remaining = warmup_deadline - time.monotonic()
                    if remaining <= 0.0:
                        raise FlightDirectorError(
                            "ten fresh hold setpoints were not available before OFFBOARD"
                        )
                    self.condition.wait(min(remaining, 0.1))

            deadline = time.monotonic() + self.args.command_timeout
            while time.monotonic() < deadline:
                self._check_abort()
                state, _ = self._state_snapshot()
                if state is None or not state.connected or not state.armed:
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
        with self.condition:
            while self.attitude_setpoint_count - attitude_baseline < 5:
                self._check_abort()
                state = self.state
                if (
                    state is None
                    or not state.connected
                    or not state.armed
                    or state.mode != "OFFBOARD"
                ):
                    raise FlightDirectorError(
                        "PX4 left armed OFFBOARD before SE3 output was verified",
                        EXIT_MANUAL_TAKEOVER,
                    )
                remaining = attitude_deadline - time.monotonic()
                if remaining <= 0.0:
                    raise FlightDirectorError(
                        "five SE3 attitude/thrust setpoints were not observed after OFFBOARD"
                    )
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
        if disarmed_mode_requires_stabilized(state.mode):
            self._reset_disarmed_to_stabilized()
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
            if state is None or not state.connected:
                raise FlightDirectorError("PX4 disconnected before mission landing")
            if not state.armed:
                self.rospy.loginfo("Vehicle is already disarmed after mission.")
                if disarmed_mode_requires_stabilized(state.mode):
                    self._reset_disarmed_to_stabilized()
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
            if state is None or not state.connected:
                raise FlightDirectorError("PX4 disconnected during mission landing")
            if not state.armed:
                self.rospy.loginfo(
                    "PX4 landing completed and the vehicle is disarmed."
                )
                if disarmed_mode_requires_stabilized(state.mode):
                    self._reset_disarmed_to_stabilized()
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

    def run(self):
        try:
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
                self.runner._request_recovery(str(exc))
            return exc.code
        finally:
            self._restore_altitude_acceptance_radius()


def _build_parser():
    parser = argparse.ArgumentParser(
        description="Execute the shared simulation/real PX4 waypoint mission"
    )
    parser.add_argument("mission_file")
    parser.add_argument("--drone-id", type=int, default=0)
    parser.add_argument("--default-takeoff-height", type=float, default=1.0)
    parser.add_argument("--preflight-timeout", type=float, default=5.0)
    parser.add_argument("--command-timeout", type=float, default=15.0)
    parser.add_argument("--takeoff-timeout", type=float, default=30.0)
    parser.add_argument("--landing-timeout", type=float, default=120.0)
    parser.add_argument("--takeoff-tolerance", type=float, default=0.1)
    parser.add_argument("--takeoff-stable-time", type=float, default=0.5)
    parser.add_argument(
        "--takeoff-max-vertical-speed", type=float, default=0.2
    )
    parser.add_argument("--odom-timeout", type=float, default=0.5)
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
        "odom_timeout",
    ):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0.0:
            print("[ERROR] {} must be finite and positive".format(name), file=sys.stderr)
            return EXIT_MISSION_FAILED

    try:
        import rospy

        rospy.init_node("shared_waypoint_mission", disable_signals=True)
        ground = float(
            rospy.get_param(
                "/drone_{}_diff_planner_node/grid_map/virtual_ground".format(
                    args.drone_id
                )
            )
        )
        ceil = float(
            rospy.get_param(
                "/drone_{}_diff_planner_node/grid_map/virtual_ceil".format(
                    args.drone_id
                )
            )
        )
        config = load_mission_config(
            args.mission_file,
            default_takeoff_height=args.default_takeoff_height,
            virtual_ground=ground,
            virtual_ceil=ceil,
        )
    except (ImportError, KeyError, ValueError, MissionConfigError) as exc:
        print("[ERROR] Cannot initialize mission: {}".format(exc), file=sys.stderr)
        return EXIT_MISSION_FAILED

    if args.takeoff_tolerance >= config["takeoff_height"]:
        print(
            "[ERROR] takeoff_tolerance must be below mission takeoff_height",
            file=sys.stderr,
        )
        return EXIT_MISSION_FAILED

    executor = SharedMissionExecutor(rospy, config, args)
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
