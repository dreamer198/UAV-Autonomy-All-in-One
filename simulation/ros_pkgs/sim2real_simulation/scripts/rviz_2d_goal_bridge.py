#!/usr/bin/env python3

"""Bridge RViz's planar goal tool to the selected planner's 3D interface.

This simulation-only adapter assigns a configured flight altitude to the 2D
goal. Arming and the OFFBOARD handoff belong exclusively to the shared
``sim.sh arm`` or mission workflow.
"""

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
    if not all(
        math.isfinite(float(value)) for value in (ground, ceil, inflation)
    ):
        raise ValueError("Planner vertical clearance values must be finite")
    if inflation < 0.0:
        raise ValueError("Planner obstacle inflation cannot be negative")
    minimum_z = ground + inflation
    maximum_z = ceil - inflation
    if minimum_z >= maximum_z:
        raise ValueError("Planner vertical fence has no usable clearance")
    return minimum_z, maximum_z


def wait_for_nonzero_ros_time(timeout):
    """Wait for the first /clock sample without sleeping in simulated time."""
    deadline = time.monotonic() + timeout
    while not rospy.is_shutdown():
        if not rospy.Time.now().is_zero():
            return
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise ValueError(
                "ROS time is still zero; RViz goal validation cannot start"
            )
        # rospy.sleep() can block forever while simulated time is still zero.
        time.sleep(min(0.02, remaining))
    raise rospy.ROSInterruptException("ROS shutdown while waiting for time")


class Rviz2DGoalBridge:
    def __init__(self):
        rospy.init_node("rviz_2d_goal_bridge")
        from sim2real_planning_msgs.msg import PlannerGoal, PlannerStatus
        from sim2real_planning_msgs.srv import ValidateGoal

        self.PlannerGoal = PlannerGoal
        self.PlannerStatus = PlannerStatus

        self.input_topic = rospy.get_param(
            "~input_topic", "/sim2real/rviz_goal"
        )
        self.output_topic = rospy.get_param("~output_topic", "/goal")
        self.frame_id = str(
            rospy.get_param("~frame_id", "world")
        ).lstrip("/")
        self.goal_z = float(rospy.get_param("~goal_z", 1.0))
        self.state_timeout = float(rospy.get_param("~state_timeout", 3.0))
        self.validate_goal_service = rospy.get_param(
            "~validate_goal_service", "/planning/validate_goal"
        )
        if not math.isfinite(self.goal_z) or self.goal_z <= 0.0:
            raise ValueError("~goal_z must be positive for an aerial goal")
        if not math.isfinite(self.state_timeout) or self.state_timeout <= 0.0:
            raise ValueError("~state_timeout must be finite and positive")
        if self.frame_id != "world":
            raise ValueError("~frame_id must be 'world'")

        self._lock = threading.Lock()
        self._state = None
        self._state_received_at = 0.0
        self._planner_status = None
        self._planner_status_received_at = 0.0
        self._goal_pub = rospy.Publisher(
            self.output_topic, PoseStamped, queue_size=1, latch=False
        )

        rospy.Subscriber(
            "/mavros/state", State, self._state_cb, queue_size=1
        )
        rospy.Subscriber(
            "/planning/status",
            PlannerStatus,
            self._planner_status_cb,
            queue_size=1,
        )
        rospy.Subscriber(
            self.input_topic, PoseStamped, self._goal_cb, queue_size=1
        )
        self._validate_goal = rospy.ServiceProxy(
            self.validate_goal_service, ValidateGoal
        )
        rospy.wait_for_service(
            self.validate_goal_service, timeout=self.state_timeout
        )
        wait_for_nonzero_ros_time(self.state_timeout)
        goal_z_error = self._goal_error(0.0, 0.0, None)
        if goal_z_error:
            raise ValueError(goal_z_error)

        rospy.loginfo(
            "RViz 2D goal bridge ready: %s -> %s, z=%.2f m; "
            "armed OFFBOARD is required",
            self.input_topic,
            self.output_topic,
            self.goal_z,
        )

    def _state_cb(self, msg):
        with self._lock:
            self._state = msg
            self._state_received_at = time.monotonic()

    def _planner_status_cb(self, msg):
        with self._lock:
            self._planner_status = msg
            self._planner_status_received_at = time.monotonic()

    def _goal_error(self, x, y, orientation):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        if pose.header.stamp.is_zero():
            return "ROS time is not initialized"
        pose.header.frame_id = "world"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = self.goal_z
        constrain_yaw = orientation is not None
        if constrain_yaw:
            pose.pose.orientation = copy.deepcopy(orientation)
        goal = self.PlannerGoal()
        goal.header.stamp = pose.header.stamp
        goal.session_id = "rviz-validation"
        goal.goal_id = 1
        goal.action = goal.PLAN
        goal.goal = pose
        goal.constrain_yaw = constrain_yaw
        try:
            response = self._validate_goal(goal)
        except Exception as exc:
            return "selected planner validation failed: {}".format(exc)
        if not response.valid:
            return response.reason or "selected planner rejected the goal"
        return ""

    def _goal_cb(self, msg):
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
        input_frame = msg.header.frame_id.lstrip("/") if msg.header.frame_id else ""
        if input_frame and input_frame != "world":
            rospy.logerr(
                "Ignoring RViz goal in frame '%s'; Planner requires 'world'.",
                msg.header.frame_id,
            )
            return

        with self._lock:
            state_ready = (
                self._state is not None
                and time.monotonic() - self._state_received_at
                <= self.state_timeout
                and self._state.connected
                and self._state.armed
                and self._state.mode == "OFFBOARD"
            )
            planner_ready = (
                self._planner_status is not None
                and time.monotonic() - self._planner_status_received_at
                <= self.state_timeout
                and self._planner_status.state != self.PlannerStatus.FAULT
                and self._planner_status.odom_ready
                and self._planner_status.map_ready
            )
        if not state_ready:
            rospy.logwarn(
                "Ignoring the RViz goal while SITL is not armed in OFFBOARD. "
                "Run ./launch/sim.sh arm first."
            )
            return
        if not planner_ready:
            rospy.logwarn(
                "Ignoring the RViz goal while the selected planner is not ready."
            )
            return
        try:
            fault_reason = localization_fault_reason(
                rospy.get_param(LOCALIZATION_FAULT_PARAM, "")
            )
        except Exception as exc:
            rospy.logerr(
                "Ignoring RViz goal because the localization interlock "
                "cannot be read: %s",
                exc,
            )
            return
        if fault_reason:
            rospy.logerr(
                "Ignoring RViz goal because localization is unsafe: %s",
                fault_reason,
            )
            return
        goal_z_error = self._goal_error(
            msg.pose.position.x, msg.pose.position.y, msg.pose.orientation
        )
        if goal_z_error:
            rospy.logerr("Ignoring RViz goal: %s", goal_z_error)
            return

        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.frame_id or "world"
        if goal.header.frame_id != "world":
            rospy.logerr(
                "Ignoring RViz goal in frame '%s'; Planner requires 'world'.",
                goal.header.frame_id,
            )
            return
        goal.pose = copy.deepcopy(msg.pose)
        goal.pose.position.z = self.goal_z
        self._goal_pub.publish(goal)

        rospy.loginfo(
            "2D Nav Goal -> %s: x=%.3f y=%.3f z=%.3f",
            self.output_topic,
            goal.pose.position.x,
            goal.pose.position.y,
            goal.pose.position.z,
        )


if __name__ == "__main__":
    try:
        Rviz2DGoalBridge()
        rospy.spin()
    except (rospy.ROSInterruptException, ValueError) as exc:
        rospy.logerr(str(exc))
