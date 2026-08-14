#!/usr/bin/env python3
"""Forward validated FAST-LIO odometry through MAVROS' ODOMETRY plugin.

The MAVROS odometry plugin emits MAVLink ODOMETRY with
MAV_FRAME_LOCAL_FRD.  Unlike VISION_POSITION_ESTIMATE, that frame tells PX4
that the external estimator has an arbitrary, locally initialized heading.
PX4 can then rotate the measurement into its EKF earth frame instead of
silently treating the FAST-LIO axes as magnetic NED.
"""

import copy
import math

import rospy
from nav_msgs.msg import Odometry


def timestamp_age_reason(stamp, now, max_age, future_tolerance):
    values = (stamp, now, max_age, future_tolerance)
    if not all(math.isfinite(float(value)) for value in values):
        return "timestamp is non-finite"
    if stamp <= 0.0:
        return "timestamp is zero or negative"
    if now <= 0.0:
        return "ROS time is not initialized"
    age = now - stamp
    if age > max_age:
        return "timestamp is {:.3f}s old (limit {:.3f}s)".format(age, max_age)
    if age < -future_tolerance:
        return "timestamp is {:.3f}s in the future".format(-age)
    return ""


def odometry_rejection_reason(
    msg,
    now,
    max_age,
    future_tolerance,
    expected_frame_id,
    expected_child_frame_id,
):
    if msg.header.frame_id != expected_frame_id:
        return "parent frame is {!r}, expected {!r}".format(
            msg.header.frame_id, expected_frame_id
        )
    if msg.child_frame_id != expected_child_frame_id:
        return "child frame is {!r}, expected {!r}".format(
            msg.child_frame_id, expected_child_frame_id
        )

    try:
        stamp = float(msg.header.stamp.to_sec())
    except (AttributeError, TypeError, ValueError, OverflowError) as exc:
        return "timestamp is unreadable: {}".format(exc)
    reason = timestamp_age_reason(stamp, now, max_age, future_tolerance)
    if reason:
        return reason

    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    linear = msg.twist.twist.linear
    angular = msg.twist.twist.angular
    values = (
        position.x,
        position.y,
        position.z,
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
        linear.x,
        linear.y,
        linear.z,
        angular.x,
        angular.y,
        angular.z,
    )
    if not all(math.isfinite(float(value)) for value in values):
        return "pose or twist contains a non-finite value"

    quaternion_norm = math.sqrt(
        orientation.x * orientation.x
        + orientation.y * orientation.y
        + orientation.z * orientation.z
        + orientation.w * orientation.w
    )
    if abs(quaternion_norm - 1.0) > 0.01:
        return "orientation quaternion norm is {:.6f}".format(quaternion_norm)
    return ""


class OdomToMavros:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/localization/odom")
        self.output_topic = rospy.get_param("~output_topic", "/mavros/odometry/out")
        self.expected_frame_id = rospy.get_param("~expected_frame_id", "world")
        self.expected_child_frame_id = rospy.get_param(
            "~expected_child_frame_id", "base_link"
        )
        self.max_input_age = float(rospy.get_param("~max_input_age", 0.2))
        self.max_future_skew = float(rospy.get_param("~max_future_skew", 0.05))
        for name, value in (
            ("max_input_age", self.max_input_age),
            ("max_future_skew", self.max_future_skew),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("~{} must be finite and positive".format(name))

        self.last_stamp = None
        self.pub = rospy.Publisher(self.output_topic, Odometry, queue_size=10)
        self.sub = rospy.Subscriber(
            self.input_topic, Odometry, self.callback, queue_size=20
        )
        rospy.loginfo(
            "external odometry bridge: %s (%s -> %s) -> %s [MAV_FRAME_LOCAL_FRD]",
            self.input_topic,
            self.expected_frame_id,
            self.expected_child_frame_id,
            self.output_topic,
        )

    def callback(self, msg):
        try:
            now = float(rospy.Time.now().to_sec())
        except (AttributeError, TypeError, ValueError, OverflowError) as exc:
            rospy.logerr_throttle(
                1.0, "external odometry bridge cannot read ROS time: %s", exc
            )
            return

        reason = odometry_rejection_reason(
            msg,
            now,
            self.max_input_age,
            self.max_future_skew,
            self.expected_frame_id,
            self.expected_child_frame_id,
        )
        if reason:
            rospy.logwarn_throttle(
                1.0, "external odometry bridge rejected input: %s", reason
            )
            return

        stamp = float(msg.header.stamp.to_sec())
        if self.last_stamp is not None and stamp <= self.last_stamp:
            rospy.logwarn_throttle(
                1.0,
                "external odometry bridge rejected a repeated or out-of-order timestamp",
            )
            return

        # Preserve the measurement, its covariance and its original frame IDs.
        # frame_aliases.launch explicitly defines world == odom so MAVROS can
        # transform world/base_link into LOCAL_FRD/BODY_FRD without relabelling.
        self.last_stamp = stamp
        self.pub.publish(copy.deepcopy(msg))


def main():
    rospy.init_node("external_odometry_bridge")
    OdomToMavros()
    rospy.spin()


if __name__ == "__main__":
    main()
