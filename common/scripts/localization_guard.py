#!/usr/bin/env python3
"""Latch localization failures and land autonomous flights immediately."""

import math
import threading
import time


LOCALIZATION_FAULT_PARAM = "/sim2real/localization_fault"
PROTECTED_AUTONOMOUS_MODES = (
    "AUTO.LOITER",
    "AUTO.TAKEOFF",
    "OFFBOARD",
)


def localization_fault_reason(value):
    """Return the stored fault reason, or an empty string when it is clear."""
    if isinstance(value, dict):
        if not value.get("active", False):
            return ""
        return str(value.get("reason") or "localization safety fault")
    if value:
        return str(value)
    return ""


def odometry_sanity_reason(message, previous_position, max_speed, max_jump):
    """Reject values that cannot represent the configured vehicle motion."""
    pose = message.pose.pose
    twist = message.twist.twist
    position = (pose.position.x, pose.position.y, pose.position.z)
    quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    velocity = (twist.linear.x, twist.linear.y, twist.linear.z)
    if not all(
        math.isfinite(float(value))
        for value in position + quaternion + velocity
    ):
        return "localization odometry contains a non-finite value"

    quaternion_norm = math.sqrt(sum(value * value for value in quaternion))
    if quaternion_norm < 0.5 or quaternion_norm > 1.5:
        return "localization odometry contains an invalid orientation"

    speed = math.sqrt(sum(value * value for value in velocity))
    if speed > max_speed:
        return (
            "localization odometry reports implausible speed "
            "{:.2f} m/s (limit {:.2f} m/s)"
        ).format(speed, max_speed)

    if previous_position is not None:
        jump = math.sqrt(
            sum(
                (position[index] - previous_position[index]) ** 2
                for index in range(3)
            )
        )
        if jump > max_jump:
            return (
                "localization odometry jumped {:.2f} m between messages "
                "(limit {:.2f} m)"
            ).format(jump, max_jump)
    return ""


class LocalizationGuard:
    """A stack-lifetime guard shared unchanged by simulation and real flight."""

    def __init__(self, rospy):
        from mavros_msgs.msg import State
        from mavros_msgs.srv import SetMode
        from nav_msgs.msg import Odometry

        self.rospy = rospy
        self.lock = threading.Lock()
        self.odom_timeout = float(rospy.get_param("~odom_timeout", 0.5))
        self.max_speed = float(rospy.get_param("~max_speed", 3.0))
        self.max_jump = float(rospy.get_param("~max_jump", 2.0))
        self.odometry_topic = rospy.get_param(
            "~odometry_topic", "/localization/odom"
        )
        self.state_topic = rospy.get_param("~state_topic", "/mavros/state")
        for name, value in (
            ("odom_timeout", self.odom_timeout),
            ("max_speed", self.max_speed),
            ("max_jump", self.max_jump),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("~{} must be finite and positive".format(name))

        self.state = None
        self.last_healthy_odom_at = None
        self.previous_position = None
        self.fault_reason = localization_fault_reason(
            rospy.get_param(LOCALIZATION_FAULT_PARAM, "")
        )
        self.last_land_request_at = 0.0
        self.set_mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        rospy.Subscriber(
            self.odometry_topic,
            Odometry,
            self._odom_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            self.state_topic, State, self._state_callback, queue_size=1
        )
        rospy.Timer(rospy.Duration.from_sec(0.05), self._timer_callback)

        if self.fault_reason:
            rospy.logerr(
                "Localization safety interlock is already latched: %s. "
                "Restart the complete stack before another autonomous flight.",
                self.fault_reason,
            )
        else:
            rospy.loginfo(
                "Localization guard active: odom timeout=%.2f s, "
                "max speed=%.2f m/s, max jump=%.2f m.",
                self.odom_timeout,
                self.max_speed,
                self.max_jump,
            )

    def _state_callback(self, message):
        with self.lock:
            self.state = message

    def _odom_callback(self, message):
        position = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.position.z,
        )
        with self.lock:
            previous_position = self.previous_position
        reason = odometry_sanity_reason(
            message, previous_position, self.max_speed, self.max_jump
        )
        if reason:
            self._latch_fault(reason)
            return
        with self.lock:
            self.previous_position = position
            self.last_healthy_odom_at = time.monotonic()

    def _latch_fault(self, reason):
        with self.lock:
            if self.fault_reason:
                return
            self.fault_reason = reason
        try:
            self.rospy.set_param(
                LOCALIZATION_FAULT_PARAM,
                {"active": True, "reason": reason},
            )
        except Exception as exc:
            self.rospy.logerr(
                "Could not persist the localization safety interlock: %s", exc
            )
        self.rospy.logerr(
            "LOCALIZATION SAFETY FAULT: %s. Autonomous commands are locked "
            "until the complete stack is restarted.",
            reason,
        )

    def _request_land_if_needed(self, now):
        with self.lock:
            state = self.state
            fault_reason = self.fault_reason
            last_request = self.last_land_request_at
        if (
            not fault_reason
            or state is None
            or not state.connected
            or not state.armed
            or state.mode not in PROTECTED_AUTONOMOUS_MODES
            or now - last_request < 0.2
        ):
            return
        with self.lock:
            self.last_land_request_at = now
        try:
            response = self.set_mode(base_mode=0, custom_mode="AUTO.LAND")
        except self.rospy.ServiceException as exc:
            self.rospy.logerr_throttle(
                1.0, "Localization guard cannot request AUTO.LAND: %s", exc
            )
            return
        if response.mode_sent:
            self.rospy.logerr_throttle(
                1.0,
                "Localization is unsafe; requested PX4 AUTO.LAND.",
            )
        else:
            self.rospy.logerr_throttle(
                1.0, "PX4 rejected localization-guard AUTO.LAND."
            )

    def _timer_callback(self, _event):
        now = time.monotonic()
        with self.lock:
            last_healthy_odom_at = self.last_healthy_odom_at
            fault_reason = self.fault_reason
        if (
            not fault_reason
            and last_healthy_odom_at is not None
            and now - last_healthy_odom_at > self.odom_timeout
        ):
            self._latch_fault(
                "localization odometry stream stopped for {:.2f} s "
                "(limit {:.2f} s)".format(
                    now - last_healthy_odom_at, self.odom_timeout
                )
            )
        self._request_land_if_needed(now)


def main():
    import rospy

    rospy.init_node("localization_guard")
    LocalizationGuard(rospy)
    rospy.spin()


if __name__ == "__main__":
    main()
