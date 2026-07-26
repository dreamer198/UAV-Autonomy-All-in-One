#!/usr/bin/env python3
"""Relay registered clouds, applying a real TF instead of relabelling frames."""

import math

import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class CloudRelay:
    def __init__(self):
        input_topic = rospy.get_param("~input_topic", "/cloud_registered")
        output_topic = rospy.get_param(
            "~output_topic", "/localization/cloud_registered"
        )
        self.target_frame = rospy.get_param("~frame_id", "world")
        self.lookup_timeout = float(rospy.get_param("~lookup_timeout", 0.1))
        if not math.isfinite(self.lookup_timeout) or self.lookup_timeout <= 0.0:
            raise ValueError("~lookup_timeout must be finite and positive")
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.publisher = rospy.Publisher(output_topic, PointCloud2, queue_size=1)
        self.subscriber = rospy.Subscriber(
            input_topic, PointCloud2, self.callback, queue_size=1
        )
        rospy.loginfo(
            "cloud_relay: %s -> %s (TF target=%s)",
            input_topic,
            output_topic,
            self.target_frame or "preserve",
        )

    def callback(self, msg):
        source_frame = msg.header.frame_id.lstrip("/")
        target_frame = self.target_frame.lstrip("/")
        if not target_frame or source_frame == target_frame:
            self.publisher.publish(msg)
            return
        if not source_frame:
            rospy.logerr_throttle(
                1.0, "cloud_relay dropped a cloud without frame_id."
            )
            return
        if msg.header.stamp == rospy.Time():
            rospy.logerr_throttle(
                1.0,
                "cloud_relay dropped a cloud without a measurement timestamp.",
            )
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                msg.header.stamp,
                rospy.Duration(self.lookup_timeout),
            )
            output = do_transform_cloud(msg, transform)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            rospy.logwarn_throttle(
                1.0,
                "cloud_relay dropped a cloud because measurement-time TF "
                "%s -> %s is unavailable: %s",
                source_frame,
                target_frame,
                exc,
            )
            return
        output.header.frame_id = target_frame
        self.publisher.publish(output)


if __name__ == "__main__":
    rospy.init_node("cloud_relay")
    CloudRelay()
    rospy.spin()
