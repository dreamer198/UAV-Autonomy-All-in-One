#!/usr/bin/env python3
"""Normalize selected-plugin visualization into stable public ROS topics."""

import copy
import math
import struct
import threading

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField
from sim2real_planning_msgs.msg import (
    PlannerCapabilities,
    PlannerStatus,
)
from visualization_msgs.msg import Marker

from sim2real_planner_manager.visualization import (
    finite_point_records,
    fixed_bounds_edges,
)


_FLOAT_FORMATS = {
    PointField.FLOAT32: "f",
    PointField.FLOAT64: "d",
}


class PlannerVisualization:
    def __init__(self):
        self.world_frame = str(
            rospy.get_param("~world_frame", "world")
        ).lstrip("/")
        self.bounds_line_width = float(
            rospy.get_param("~bounds_line_width", 0.045)
        )
        self.publish_rate = float(rospy.get_param("~publish_rate", 1.0))
        self.max_occupancy_points = int(
            rospy.get_param("~max_occupancy_points", 20000)
        )
        self.max_inflated_points = int(
            rospy.get_param("~max_inflated_points", 12000)
        )
        if not self.world_frame:
            raise ValueError("~world_frame must not be empty")
        if (
            not math.isfinite(self.bounds_line_width)
            or self.bounds_line_width <= 0.0
        ):
            raise ValueError("visualization line width must be positive")
        if not math.isfinite(self.publish_rate) or self.publish_rate <= 0.0:
            raise ValueError("visualization publish rate must be positive")
        if self.max_occupancy_points < 1 or self.max_inflated_points < 1:
            raise ValueError("visualization point caps must be positive")

        self.lock = threading.RLock()
        self.map_ready = False
        self.maps_cleared = True
        self.map_templates = {"occupancy": None, "inflated": None}
        self.map_dirty = {"occupancy": False, "inflated": False}

        self.occupancy_pub = rospy.Publisher(
            rospy.get_param(
                "~occupancy_topic", "/planning/viz/occupancy"
            ),
            PointCloud2,
            queue_size=1,
            latch=True,
        )
        self.inflated_pub = rospy.Publisher(
            rospy.get_param(
                "~inflated_topic", "/planning/viz/inflated_occupancy"
            ),
            PointCloud2,
            queue_size=1,
            latch=True,
        )
        self.bounds_pub = rospy.Publisher(
            rospy.get_param(
                "~bounds_topic", "/planning/viz/planning_bounds"
            ),
            Marker,
            queue_size=1,
            latch=True,
        )
        common_subscriber_options = {
            "queue_size": 1,
            "buff_size": 16 * 1024 * 1024,
        }
        self.raw_occupancy_sub = rospy.Subscriber(
            rospy.get_param(
                "~raw_occupancy_topic", "/planning/viz/raw/occupancy"
            ),
            PointCloud2,
            lambda message: self.map_callback("occupancy", message),
            **common_subscriber_options
        )
        self.raw_inflated_sub = rospy.Subscriber(
            rospy.get_param(
                "~raw_inflated_topic",
                "/planning/viz/raw/inflated_occupancy",
            ),
            PointCloud2,
            lambda message: self.map_callback("inflated", message),
            **common_subscriber_options
        )
        self.capabilities_sub = rospy.Subscriber(
            rospy.get_param(
                "~capabilities_topic", "/planning/capabilities"
            ),
            PlannerCapabilities,
            self.capabilities_callback,
            queue_size=1,
        )
        self.status_sub = rospy.Subscriber(
            rospy.get_param("~status_topic", "/planning/status"),
            PlannerStatus,
            self.status_callback,
            queue_size=5,
        )
        self.publish_timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate), self.publish_maps
        )
        rospy.loginfo(
            "Planner-neutral visualization ready: finite raw map points -> "
            "normalized maps, capabilities -> bounds.",
        )

    def coordinate_specs(self, message):
        specs = {}
        for field in message.fields:
            if field.name not in ("x", "y", "z"):
                continue
            if field.count != 1 or field.datatype not in _FLOAT_FORMATS:
                raise ValueError(
                    "{} must be a scalar float field".format(field.name)
                )
            specs[field.name] = (
                field.offset,
                _FLOAT_FORMATS[field.datatype],
            )
        if set(specs) != {"x", "y", "z"}:
            raise ValueError("point cloud must contain scalar float XYZ fields")
        return specs

    def normalized_cloud(self, message):
        if message.header.frame_id.lstrip("/") != self.world_frame:
            raise ValueError(
                "point-cloud frame must be {}".format(self.world_frame)
            )
        data, kept = finite_point_records(
            message.data,
            message.width,
            message.height,
            message.point_step,
            message.row_step,
            self.coordinate_specs(message),
            message.is_bigendian,
        )
        output = PointCloud2()
        output.header = copy.deepcopy(message.header)
        if output.header.stamp == rospy.Time():
            output.header.stamp = rospy.Time.now()
        output.header.frame_id = self.world_frame
        output.height = 1
        output.width = kept
        output.fields = copy.deepcopy(message.fields)
        output.is_bigendian = message.is_bigendian
        output.point_step = message.point_step
        output.row_step = kept * message.point_step
        output.data = data
        output.is_dense = True
        return output

    @staticmethod
    def empty_cloud(template):
        output = copy.deepcopy(template)
        output.header.stamp = rospy.Time.now()
        output.height = 1
        output.width = 0
        output.row_step = 0
        output.data = b""
        output.is_dense = True
        return output

    @staticmethod
    def bounded_cloud(message, max_points):
        """Deterministically thin a normalized cloud without changing fields."""
        if message.width <= max_points:
            return message
        stride = int(math.ceil(float(message.width) / float(max_points)))
        records = [
            message.data[offset : offset + message.point_step]
            for offset in range(0, message.row_step, stride * message.point_step)
        ]
        output = copy.deepcopy(message)
        output.width = len(records)
        output.row_step = output.width * output.point_step
        output.data = b"".join(records)
        return output

    def map_callback(self, kind, message):
        try:
            output = self.normalized_cloud(message)
        except (TypeError, ValueError, struct.error) as exc:
            rospy.logwarn_throttle(
                5.0, "Rejected raw planner %s map: %s", kind, exc
            )
            return
        with self.lock:
            cap = (
                self.max_occupancy_points
                if kind == "occupancy"
                else self.max_inflated_points
            )
            self.map_templates[kind] = self.bounded_cloud(output, cap)
            self.map_dirty[kind] = True

    def publish_maps(self, _event):
        with self.lock:
            if not self.map_ready:
                return
            pending = {
                kind: self.map_templates[kind]
                for kind in ("occupancy", "inflated")
                if self.map_dirty[kind] and self.map_templates[kind] is not None
            }
            for kind in pending:
                self.map_dirty[kind] = False
            if pending:
                self.maps_cleared = False
            # Serialize map publication with a map-ready loss.  Otherwise a
            # timer callback could publish an old pending cloud immediately
            # after clear_maps() latched an empty one.
            if "occupancy" in pending:
                self.occupancy_pub.publish(pending["occupancy"])
            if "inflated" in pending:
                self.inflated_pub.publish(pending["inflated"])

    def clear_maps(self):
        with self.lock:
            if self.maps_cleared:
                return
            templates = dict(self.map_templates)
            self.map_dirty = {"occupancy": False, "inflated": False}
            self.maps_cleared = True
            if templates["occupancy"] is not None:
                self.occupancy_pub.publish(
                    self.empty_cloud(templates["occupancy"])
                )
            if templates["inflated"] is not None:
                self.inflated_pub.publish(
                    self.empty_cloud(templates["inflated"])
                )

    def delete_marker(self, publisher, namespace):
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = 0
        marker.action = Marker.DELETE
        publisher.publish(marker)

    def capabilities_callback(self, message):
        if not message.has_fixed_map_bounds:
            self.delete_marker(self.bounds_pub, "fixed_planning_bounds")
            return
        if message.header.frame_id.lstrip("/") != self.world_frame:
            rospy.logwarn(
                "Planner bounds rejected frame '%s'.",
                message.header.frame_id,
            )
            self.delete_marker(self.bounds_pub, "fixed_planning_bounds")
            return
        minimum = (
            message.map_min.x,
            message.map_min.y,
            message.map_min.z,
        )
        maximum = (
            message.map_max.x,
            message.map_max.y,
            message.map_max.z,
        )
        try:
            edges = fixed_bounds_edges(minimum, maximum)
        except ValueError as exc:
            rospy.logwarn("Planner bounds rejected: %s", exc)
            self.delete_marker(self.bounds_pub, "fixed_planning_bounds")
            return

        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "fixed_planning_bounds"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.bounds_line_width
        marker.color.r = 0.1
        marker.color.g = 0.85
        marker.color.b = 1.0
        marker.color.a = 0.95
        marker.points = [Point(x=x, y=y, z=z) for x, y, z in edges]
        marker.frame_locked = True
        marker.lifetime = rospy.Duration()
        self.bounds_pub.publish(marker)

    def status_callback(self, message):
        map_ready = bool(
            message.map_ready and message.state != PlannerStatus.FAULT
        )
        with self.lock:
            lost_map = self.map_ready and not map_ready
            self.map_ready = map_ready
            if lost_map:
                # RLock keeps the readiness transition and latched clears in
                # one ordered critical section.
                self.clear_maps()


if __name__ == "__main__":
    rospy.init_node("planner_visualization")
    try:
        PlannerVisualization()
    except (TypeError, ValueError) as exc:
        rospy.logfatal("Invalid planner visualization configuration: %s", exc)
        raise
    rospy.spin()
