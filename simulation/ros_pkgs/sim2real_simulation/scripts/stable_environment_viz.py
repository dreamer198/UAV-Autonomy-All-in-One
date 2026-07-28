#!/usr/bin/env python3
"""Publish a stable, planner-independent environment layer for RViz."""

import os
import sys
import threading

import numpy as np
import rospy
import tf2_ros
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from environment_voxel_map import PersistentVoxelMap  # noqa: E402


class StableEnvironmentVisualization:
    def __init__(self):
        self.input_topic = rospy.get_param(
            "~input_topic", "/localization/cloud_registered"
        )
        self.output_topic = rospy.get_param(
            "~output_topic", "/planning/viz/environment"
        )
        self.world_frame = rospy.get_param("~world_frame", "world").lstrip("/")
        self.sensor_frame = rospy.get_param(
            "~sensor_frame", "livox_link"
        ).lstrip("/")
        self.lookup_timeout = float(rospy.get_param("~lookup_timeout", 0.05))
        self.publish_rate = float(rospy.get_param("~publish_rate", 2.0))
        self.log_interval = float(rospy.get_param("~log_interval", 5.0))

        if not np.isfinite(self.publish_rate) or self.publish_rate <= 0.0:
            raise ValueError("~publish_rate must be finite and positive")
        if not self.world_frame or not self.sensor_frame:
            raise ValueError("~world_frame and ~sensor_frame must not be empty")

        self.map = PersistentVoxelMap(
            voxel_size=rospy.get_param("~voxel_size", 0.12),
            min_range=rospy.get_param("~min_range", 0.2),
            max_range=rospy.get_param("~max_range", 44.0),
            min_z=rospy.get_param("~min_z", 0.15),
            max_z=rospy.get_param("~max_z", 3.2),
            min_hits=rospy.get_param("~min_hits", 2),
            max_voxels=rospy.get_param("~max_voxels", 300000),
        )

        self.lock = threading.Lock()
        self.last_stamp = rospy.Time()
        self.tf_failures = 0
        self.invalid_clouds = 0

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.environment_pub = rospy.Publisher(
            self.output_topic, PointCloud2, queue_size=1, latch=True
        )
        self.cloud_sub = rospy.Subscriber(
            self.input_topic, PointCloud2, self.cloud_callback, queue_size=1
        )
        self.publish_timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate), self.publish_map
        )

        rospy.loginfo(
            "Stable RViz environment: %s -> %s "
            "(voxel=%.2fm, range=[%.1f, %.1f]m, z=[%.2f, %.2f]m). "
            "This layer is visualization-only and is never fed to a planner.",
            self.input_topic,
            self.output_topic,
            self.map.voxel_size,
            self.map.min_range,
            self.map.max_range,
            self.map.min_z,
            self.map.max_z,
        )

    def cloud_callback(self, msg):
        frame_id = msg.header.frame_id.lstrip("/")
        if (
            frame_id != self.world_frame
            or msg.header.stamp == rospy.Time()
        ):
            self.invalid_clouds += 1
            rospy.logwarn_throttle(
                self.log_interval,
                "Stable RViz map rejected cloud with frame='%s', stamp=%.3f "
                "(expected nonzero %s-frame data).",
                frame_id,
                msg.header.stamp.to_sec(),
                self.world_frame,
            )
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.sensor_frame,
                msg.header.stamp,
                rospy.Duration(self.lookup_timeout),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            self.tf_failures += 1
            rospy.logwarn_throttle(
                self.log_interval,
                "Stable RViz map is waiting for measurement-time lidar TF: %s "
                "(failures=%d).",
                exc,
                self.tf_failures,
            )
            return

        points = np.asarray(
            list(
                pc2.read_points(
                    msg, field_names=("x", "y", "z"), skip_nans=False
                )
            ),
            dtype=np.float32,
        )
        sensor_origin = np.asarray(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            dtype=np.float32,
        )
        with self.lock:
            self.map.add(points, sensor_origin)
            self.last_stamp = msg.header.stamp

    def publish_map(self, _event):
        with self.lock:
            points = self.map.points()
            last_stamp = self.last_stamp
            stored = self.map.voxel_count
            dropped = self.map.total_dropped_voxels
        if points.size == 0 or last_stamp == rospy.Time():
            return

        header = Header(frame_id=self.world_frame, stamp=last_stamp)
        self.environment_pub.publish(pc2.create_cloud_xyz32(header, points))
        rospy.loginfo_throttle(
            self.log_interval,
            "Stable RViz map: %d visible voxels (%d stored, %d dropped at cap).",
            points.shape[0],
            stored,
            dropped,
        )


if __name__ == "__main__":
    rospy.init_node("stable_environment_viz")
    try:
        StableEnvironmentVisualization()
    except (TypeError, ValueError) as exc:
        rospy.logfatal("Invalid stable RViz visualization configuration: %s", exc)
        raise
    rospy.spin()
