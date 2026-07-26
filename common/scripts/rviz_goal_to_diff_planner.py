#!/usr/bin/env python3
"""Safely bridge RViz's planar goal tool to Diff-Planner."""

import copy
import math
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State


LOCALIZATION_FAULT_PARAM = "/sim2real/localization_fault"


def localization_fault_reason(value):
    if isinstance(value, dict):
        if not value.get("active", False):
            return ""
        return str(value.get("reason") or "localization safety fault")
    return str(value) if value else ""


def vertical_clearance_bounds(ground, ceil, inflation):
    values = (ground, ceil, inflation)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("Planner vertical clearance values must be finite")
    if inflation < 0.0:
        raise ValueError("Planner obstacle inflation cannot be negative")
    minimum_z = ground + inflation
    maximum_z = ceil - inflation
    if minimum_z >= maximum_z:
        raise ValueError(
            "Planner vertical fence has no space after obstacle inflation"
        )
    return minimum_z, maximum_z


class RvizGoalToDiffPlanner:
    def __init__(self):
        rospy.init_node("rviz_goal_to_diff_planner")

        self.default_z = float(rospy.get_param("~default_z", 0.8))
        self.input_topic = rospy.get_param(
            "~input_topic", "/sim2real/rviz_goal"
        )
        self.output_topic = rospy.get_param("~output_topic", "/goal")
        self.frame_id_override = rospy.get_param("~frame_id", "")
        self.required_frame_id = rospy.get_param("~required_frame_id", "world")
        self.state_topic = rospy.get_param("~state_topic", "/mavros/state")
        self.state_timeout = float(rospy.get_param("~state_timeout", 3.0))
        if not math.isfinite(self.default_z):
            raise ValueError("~default_z must be finite")
        if not math.isfinite(self.state_timeout) or self.state_timeout <= 0.0:
            raise ValueError("~state_timeout must be finite and positive")
        if not self.required_frame_id:
            raise ValueError("~required_frame_id cannot be empty")
        if (
            self.frame_id_override
            and self.frame_id_override.lstrip("/")
            != self.required_frame_id.lstrip("/")
        ):
            raise ValueError("~frame_id must match ~required_frame_id")

        self.lock = threading.Lock()
        self.state = None
        self.state_received_at = 0.0
        self.goal_pub = rospy.Publisher(
            self.output_topic, PoseStamped, queue_size=1, latch=False
        )
        rospy.Subscriber(
            self.state_topic, State, self._state_cb, queue_size=1
        )
        rospy.Subscriber(
            self.input_topic, PoseStamped, self.nav_goal_cb, queue_size=1
        )

        startup_error = self._goal_z_error()
        if startup_error:
            raise ValueError(
                "RViz goal bridge default altitude is not usable: {}".format(
                    startup_error
                )
            )
        rospy.loginfo(
            "rviz_goal_to_diff_planner started: %s -> %s, "
            "default_z=%.3f; armed OFFBOARD is required",
            self.input_topic,
            self.output_topic,
            self.default_z,
        )

    def _state_cb(self, message):
        with self.lock:
            self.state = message
            self.state_received_at = time.monotonic()

    def resolve_frame(self, header):
        input_frame = header.frame_id.lstrip("/") if header.frame_id else ""
        required = self.required_frame_id.lstrip("/")
        if input_frame and input_frame != required:
            return ""
        return required

    def _goal_z_error(self):
        prefix = "/drone_0_diff_planner_node/grid_map"
        try:
            ground = float(
                rospy.get_param("{}/virtual_ground".format(prefix))
            )
            ceil = float(rospy.get_param("{}/virtual_ceil".format(prefix)))
            inflation = float(
                rospy.get_param("{}/obstacles_inflation".format(prefix))
            )
            minimum_z, maximum_z = vertical_clearance_bounds(
                ground, ceil, inflation
            )
        except Exception as exc:
            return "cannot read Planner vertical fence: {}".format(exc)
        if not minimum_z < self.default_z < maximum_z:
            return (
                "default Z={:.3f} must be inside the Planner clearance "
                "interval ({:.3f}, {:.3f})"
            ).format(self.default_z, minimum_z, maximum_z)
        return ""

    def _readiness_error(self):
        try:
            reason = localization_fault_reason(
                rospy.get_param(LOCALIZATION_FAULT_PARAM, "")
            )
        except Exception as exc:
            return "cannot read localization safety interlock: {}".format(exc)
        if reason:
            return "localization safety interlock is latched: {}".format(reason)
        with self.lock:
            state = self.state
            state_received_at = self.state_received_at
        if (
            state is None
            or time.monotonic() - state_received_at > self.state_timeout
        ):
            return "MAVROS state is unavailable or stale"
        if not state.connected:
            return "MAVROS is disconnected"
        if not state.armed or state.mode != "OFFBOARD":
            return "vehicle is not armed in OFFBOARD"
        return self._goal_z_error()

    def nav_goal_cb(self, msg):
        pose_values = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )
        if not all(math.isfinite(float(value)) for value in pose_values):
            rospy.logerr("Ignoring a non-finite RViz 2D goal.")
            return
        frame_id = self.resolve_frame(msg.header)
        if not frame_id:
            rospy.logerr(
                "Ignoring RViz goal in frame '%s'; Planner requires '%s'.",
                msg.header.frame_id,
                self.required_frame_id,
            )
            return
        readiness_error = self._readiness_error()
        if readiness_error:
            rospy.logwarn("Ignoring RViz goal: %s.", readiness_error)
            return

        out = PoseStamped()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = frame_id
        out.pose = copy.deepcopy(msg.pose)
        out.pose.position.z = self.default_z
        self.goal_pub.publish(out)

        rospy.loginfo(
            "2D Nav Goal -> %s: x=%.3f y=%.3f z=%.3f frame=%s",
            self.output_topic,
            out.pose.position.x,
            out.pose.position.y,
            out.pose.position.z,
            out.header.frame_id,
        )


if __name__ == "__main__":
    try:
        RvizGoalToDiffPlanner()
        rospy.spin()
    except (rospy.ROSInterruptException, ValueError) as exc:
        rospy.logerr(str(exc))
