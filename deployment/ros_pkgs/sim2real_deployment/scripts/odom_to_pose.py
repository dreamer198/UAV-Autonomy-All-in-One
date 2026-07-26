#!/usr/bin/env python3
import copy
import math
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped
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


class OdomToPose:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.publish_rate = float(rospy.get_param("~publish_rate", 30.0))
        self.max_input_age = float(rospy.get_param("~max_input_age", 0.2))
        self.max_future_skew = float(rospy.get_param("~max_future_skew", 0.05))
        self.use_input_frame_id = bool(
            rospy.get_param("~use_input_frame_id", False)
        )
        if not bool(rospy.get_param("~use_input_stamp", True)):
            rospy.logwarn(
                "odom_to_pose always preserves measurement timestamps; "
                "~use_input_stamp=false is ignored for EKF safety."
            )

        odom_topic = rospy.get_param("~odom_topic", "/localization/odom")
        pose_topic = rospy.get_param("~pose_topic", "/mavros/vision_pose/pose")

        for name, value in (
            ("publish_rate", self.publish_rate),
            ("max_input_age", self.max_input_age),
            ("max_future_skew", self.max_future_skew),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("~{} must be finite and positive".format(name))

        self.lock = threading.Lock()
        self.latest_pose = None
        self.latest_header = None
        self.latest_input_monotonic = None
        self.latest_stamp = None
        self.last_published_stamp = None

        self.pub = rospy.Publisher(pose_topic, PoseStamped, queue_size=10)
        rospy.Subscriber(odom_topic, Odometry, self.cb, queue_size=20)
        rospy.Timer(rospy.Duration.from_sec(1.0 / self.publish_rate), self.timer_cb)

        rospy.loginfo("odom_to_pose odom_topic: %s", odom_topic)
        rospy.loginfo("odom_to_pose pose_topic: %s", pose_topic)
        rospy.loginfo("odom_to_pose publish_rate: %.1f Hz", self.publish_rate)
        rospy.loginfo("odom_to_pose preserves input measurement timestamps")

    def cb(self, msg):
        try:
            stamp = float(msg.header.stamp.to_sec())
            now = float(rospy.Time.now().to_sec())
        except (AttributeError, TypeError, ValueError, OverflowError) as exc:
            rospy.logerr_throttle(
                1.0, "odom_to_pose rejected an unreadable timestamp: %s", exc
            )
            return
        reason = timestamp_age_reason(
            stamp, now, self.max_input_age, self.max_future_skew
        )
        if reason:
            rospy.logwarn_throttle(
                1.0, "odom_to_pose rejected input: %s", reason
            )
            return
        with self.lock:
            if self.latest_stamp is not None and stamp <= self.latest_stamp:
                rospy.logwarn_throttle(
                    1.0,
                    "odom_to_pose rejected a repeated or out-of-order "
                    "measurement timestamp.",
                )
                return
            self.latest_header = copy.deepcopy(msg.header)
            self.latest_pose = copy.deepcopy(msg.pose.pose)
            self.latest_input_monotonic = time.monotonic()
            self.latest_stamp = stamp

    def timer_cb(self, _event):
        with self.lock:
            pose_snapshot = copy.deepcopy(self.latest_pose)
            header_snapshot = copy.deepcopy(self.latest_header)
            input_monotonic = self.latest_input_monotonic
            latest_stamp = self.latest_stamp
            already_published = (
                latest_stamp is not None
                and latest_stamp == self.last_published_stamp
            )
        if (
            pose_snapshot is None
            or header_snapshot is None
            or input_monotonic is None
            or already_published
        ):
            return

        receipt_age = time.monotonic() - input_monotonic
        if receipt_age > self.max_input_age:
            rospy.logwarn_throttle(
                1.0,
                "odom_to_pose input receipt timeout: %.3fs",
                receipt_age,
            )
            return
        now = rospy.Time.now()
        reason = timestamp_age_reason(
            header_snapshot.stamp.to_sec(),
            now.to_sec(),
            self.max_input_age,
            self.max_future_skew,
        )
        if reason:
            rospy.logwarn_throttle(
                1.0, "odom_to_pose input measurement is stale: %s", reason
            )
            return

        pose = PoseStamped()
        pose.header.stamp = header_snapshot.stamp
        if self.frame_id:
            pose.header.frame_id = self.frame_id
        elif self.use_input_frame_id:
            pose.header.frame_id = header_snapshot.frame_id
        pose.pose = pose_snapshot
        with self.lock:
            # A newer callback may have arrived while this sample was checked;
            # mark only the snapshot being published.
            self.last_published_stamp = latest_stamp
        self.pub.publish(pose)


if __name__ == "__main__":
    rospy.init_node("odom_to_pose")
    OdomToPose()
    rospy.spin()
