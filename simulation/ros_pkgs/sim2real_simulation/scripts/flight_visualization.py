#!/usr/bin/env python3
"""Visualize measured flight history and the planner's authoritative goal."""

import copy
import math
import os
import sys
import threading

import rospy
from geometry_msgs.msg import Point
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from sim2real_planning_msgs.msg import PlannerStatus
from visualization_msgs.msg import Marker

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from flight_path_history import FlightPathHistory  # noqa: E402


class FlightVisualization:
    def __init__(self):
        self.world_frame = str(
            rospy.get_param("~world_frame", "world")
        ).lstrip("/")
        self.odom_topic = rospy.get_param(
            "~odom_topic", "/localization/odom"
        )
        self.state_topic = rospy.get_param("~state_topic", "/mavros/state")
        self.status_topic = rospy.get_param(
            "~status_topic", "/planning/status"
        )
        self.path_topic = rospy.get_param(
            "~path_topic", "/planning/viz/executed_path"
        )
        self.active_goal_topic = rospy.get_param(
            "~active_goal_marker_topic", "/planning/viz/active_goal"
        )
        self.path_line_width = float(
            rospy.get_param("~path_line_width", 0.055)
        )
        self.history = FlightPathHistory(
            min_distance=rospy.get_param("~path_min_distance", 0.03),
            max_points=rospy.get_param("~path_max_points", 5000),
        )
        if not self.world_frame:
            raise ValueError("~world_frame must not be empty")
        if (
            not math.isfinite(self.path_line_width)
            or self.path_line_width <= 0.0
        ):
            raise ValueError("~path_line_width must be finite and positive")

        self.lock = threading.Lock()
        self.active_goal_initialized = False
        self.active_goal_signature = None
        self.path_pub = rospy.Publisher(
            self.path_topic, Marker, queue_size=1, latch=True
        )
        self.active_goal_pub = rospy.Publisher(
            self.active_goal_topic, Marker, queue_size=1, latch=True
        )
        self.state_sub = rospy.Subscriber(
            self.state_topic, State, self.state_callback, queue_size=2
        )
        self.odom_sub = rospy.Subscriber(
            self.odom_topic, Odometry, self.odom_callback, queue_size=20
        )
        self.status_sub = rospy.Subscriber(
            self.status_topic,
            PlannerStatus,
            self.status_callback,
            queue_size=5,
        )

        rospy.loginfo(
            "Flight visualization ready: measured path=%s, "
            "active planner goal=%s.",
            self.path_topic,
            self.active_goal_topic,
        )

    @staticmethod
    def finite_pose(pose):
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        return all(math.isfinite(float(value)) for value in values)

    def marker_header(self, stamp):
        header = copy.deepcopy(stamp)
        if header == rospy.Time():
            header = rospy.Time.now()
        return header

    def delete_marker(self, publisher, namespace, marker_id=0):
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = namespace
        marker.id = marker_id
        marker.action = Marker.DELETE
        publisher.publish(marker)

    def state_callback(self, msg):
        with self.lock:
            started_sortie = self.history.set_armed(msg.armed)
        if started_sortie:
            self.delete_marker(self.path_pub, "measured_flight_path")

    def odom_callback(self, msg):
        if msg.header.frame_id.lstrip("/") != self.world_frame:
            rospy.logwarn_throttle(
                5.0,
                "Flight path rejected odometry in frame '%s' (expected %s).",
                msg.header.frame_id,
                self.world_frame,
            )
            return
        position = msg.pose.pose.position
        with self.lock:
            added = self.history.add(
                msg.header.stamp.to_sec(),
                (position.x, position.y, position.z),
            )
            if not added:
                return
            points = self.history.points()

        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = self.marker_header(msg.header.stamp)
        marker.ns = "measured_flight_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.path_line_width
        marker.color.r = 0.05
        marker.color.g = 0.95
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.points = [Point(x=x, y=y, z=z) for x, y, z in points]
        marker.lifetime = rospy.Duration()
        self.path_pub.publish(marker)

    def status_callback(self, msg):
        visible_states = (
            PlannerStatus.PLANNING,
            PlannerStatus.ACTIVE,
            PlannerStatus.REACHED,
        )
        goal = msg.active_goal
        frame_id = goal.header.frame_id.lstrip("/")
        if (
            msg.goal_id == 0
            or msg.state not in visible_states
            or frame_id != self.world_frame
            or not self.finite_pose(goal.pose)
        ):
            if (
                not self.active_goal_initialized
                or self.active_goal_signature is not None
            ):
                self.delete_marker(
                    self.active_goal_pub, "active_planner_goal"
                )
            self.active_goal_initialized = True
            self.active_goal_signature = None
            return

        signature = (
            msg.session_id,
            msg.goal_id,
            goal.pose.position.x,
            goal.pose.position.y,
            goal.pose.position.z,
        )
        if signature == self.active_goal_signature:
            return
        self.active_goal_initialized = True
        self.active_goal_signature = signature

        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = self.marker_header(msg.header.stamp)
        marker.ns = "active_planner_goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = copy.deepcopy(goal.pose)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.38
        marker.scale.y = 0.38
        marker.scale.z = 0.38
        marker.color.r = 0.15
        marker.color.g = 1.0
        marker.color.b = 0.25
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration()
        self.active_goal_pub.publish(marker)


if __name__ == "__main__":
    rospy.init_node("flight_visualization")
    try:
        FlightVisualization()
    except (TypeError, ValueError) as exc:
        rospy.logfatal("Invalid flight visualization configuration: %s", exc)
        raise
    rospy.spin()
