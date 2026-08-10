#!/usr/bin/env python3
"""Safely bridge RViz's planar goal tool to the selected planner."""

import copy
import math
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from std_srvs.srv import Trigger, TriggerResponse


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


def wait_for_nonzero_ros_time(timeout):
    """Wait for the active ROS clock without relying on simulated-time sleep."""
    deadline = time.monotonic() + timeout
    while not rospy.is_shutdown():
        if not rospy.Time.now().is_zero():
            return
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise ValueError(
                "ROS time is still zero; RViz goal validation cannot start"
            )
        time.sleep(min(0.02, remaining))
    raise rospy.ROSInterruptException("ROS shutdown while waiting for time")


class RvizGoalToDiffPlanner:
    def __init__(self):
        rospy.init_node("rviz_goal_to_diff_planner")
        from sim2real_planning_msgs.msg import PlannerGoal, PlannerStatus
        from sim2real_planning_msgs.srv import ValidateGoal

        self.PlannerGoal = PlannerGoal
        self.PlannerStatus = PlannerStatus

        self.default_z = float(rospy.get_param("~default_z", 0.8))
        self.input_topic = rospy.get_param(
            "~input_topic", "/sim2real/rviz_goal"
        )
        self.output_topic = rospy.get_param("~output_topic", "/goal")
        self.frame_id_override = rospy.get_param("~frame_id", "")
        self.required_frame_id = rospy.get_param("~required_frame_id", "world")
        self.state_topic = rospy.get_param("~state_topic", "/mavros/state")
        self.state_timeout = float(rospy.get_param("~state_timeout", 3.0))
        self.planner_status_topic = rospy.get_param(
            "~planner_status_topic", "/planning/status"
        )
        self.validate_goal_service = rospy.get_param(
            "~validate_goal_service", "/planning/validate_goal"
        )
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
        self.planner_status = None
        self.planner_status_received_at = 0.0
        self.goal_pub = rospy.Publisher(
            self.output_topic, PoseStamped, queue_size=1, latch=False
        )
        rospy.Subscriber(
            self.state_topic, State, self._state_cb, queue_size=1
        )
        rospy.Subscriber(
            self.planner_status_topic,
            PlannerStatus,
            self._planner_status_cb,
            queue_size=1,
        )
        rospy.Subscriber(
            self.input_topic, PoseStamped, self.nav_goal_cb, queue_size=1
        )

        self.validate_goal = rospy.ServiceProxy(
            self.validate_goal_service, ValidateGoal
        )
        rospy.wait_for_service(
            self.validate_goal_service, timeout=self.state_timeout
        )
        wait_for_nonzero_ros_time(self.state_timeout)
        startup_error = self._goal_error_at(0.0, 0.0, None)
        if startup_error:
            raise ValueError(
                "RViz goal bridge default altitude is not usable: {}".format(
                    startup_error
                )
            )
        # Advertise readiness only after the planner validation service is
        # reachable, ROS time is active, and the configured altitude has been
        # accepted. The launcher uses this service instead of treating early
        # ROS node registration as successful initialization.
        self.ready_service = rospy.Service("~ready", Trigger, self._ready_cb)
        rospy.loginfo(
            "RViz goal bridge started: %s -> %s, "
            "default_z=%.3f; armed OFFBOARD is required",
            self.input_topic,
            self.output_topic,
            self.default_z,
        )

    @staticmethod
    def _ready_cb(_request):
        return TriggerResponse(
            success=True,
            message="RViz goal bridge initialization is complete",
        )

    def _state_cb(self, message):
        with self.lock:
            self.state = message
            self.state_received_at = time.monotonic()

    def _planner_status_cb(self, message):
        with self.lock:
            self.planner_status = message
            self.planner_status_received_at = time.monotonic()

    def resolve_frame(self, header):
        input_frame = header.frame_id.lstrip("/") if header.frame_id else ""
        required = self.required_frame_id.lstrip("/")
        if input_frame and input_frame != required:
            return ""
        return required

    def _goal_error_at(self, x, y, orientation):
        goal_pose = PoseStamped()
        goal_pose.header.stamp = rospy.Time.now()
        if goal_pose.header.stamp.is_zero():
            return "ROS time is not initialized"
        goal_pose.header.frame_id = self.required_frame_id
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = self.default_z
        constrain_yaw = orientation is not None
        if constrain_yaw:
            goal_pose.pose.orientation = copy.deepcopy(orientation)
        goal = self.PlannerGoal()
        goal.header.stamp = goal_pose.header.stamp
        goal.session_id = "rviz-validation"
        goal.goal_id = 1
        goal.action = goal.PLAN
        goal.goal = goal_pose
        goal.constrain_yaw = constrain_yaw
        try:
            response = self.validate_goal(goal)
        except Exception as exc:
            return "selected planner validation failed: {}".format(exc)
        if not response.valid:
            return response.reason or "selected planner rejected the goal"
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
            planner_status = self.planner_status
            planner_status_received_at = self.planner_status_received_at
        if (
            state is None
            or time.monotonic() - state_received_at > self.state_timeout
        ):
            return "MAVROS state is unavailable or stale"
        if not state.connected:
            return "MAVROS is disconnected"
        if not state.armed or state.mode != "OFFBOARD":
            return "vehicle is not armed in OFFBOARD"
        if (
            planner_status is None
            or time.monotonic() - planner_status_received_at > self.state_timeout
        ):
            return "selected planner status is unavailable or stale"
        if planner_status.state == self.PlannerStatus.FAULT:
            return "selected planner fault: {}".format(
                planner_status.reason or "unspecified fault"
            )
        if not planner_status.odom_ready or not planner_status.map_ready:
            return "selected planner is not map/odom ready"
        return ""

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
        validation_error = self._goal_error_at(
            msg.pose.position.x, msg.pose.position.y, msg.pose.orientation
        )
        if validation_error:
            rospy.logwarn("Ignoring RViz goal: %s.", validation_error)
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
    except rospy.ROSInterruptException:
        pass
    except (rospy.ROSException, ValueError) as exc:
        rospy.logerr(str(exc))
        raise SystemExit(1)
