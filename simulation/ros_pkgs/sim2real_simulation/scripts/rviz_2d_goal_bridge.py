#!/usr/bin/env python3

"""Bridge RViz's planar goal tool to Diff-Planner's 3D goal interface.

This simulation-only adapter assigns a configured flight altitude to the 2D
goal. Arming and the OFFBOARD handoff belong exclusively to the shared
``sim.sh arm`` or mission workflow.
"""

import copy
import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State


class Rviz2DGoalBridge:
    def __init__(self):
        rospy.init_node("rviz_2d_goal_bridge")

        self.input_topic = rospy.get_param(
            "~input_topic", "/sim2real/rviz_goal"
        )
        self.output_topic = rospy.get_param("~output_topic", "/goal")
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.goal_z = float(rospy.get_param("~goal_z", 1.0))
        if not math.isfinite(self.goal_z) or self.goal_z <= 0.0:
            raise ValueError("~goal_z must be positive for an aerial goal")

        self._lock = threading.Lock()
        self._state = State()
        self._goal_pub = rospy.Publisher(
            self.output_topic, PoseStamped, queue_size=1, latch=False
        )

        rospy.Subscriber(
            "/mavros/state", State, self._state_cb, queue_size=1
        )
        rospy.Subscriber(
            self.input_topic, PoseStamped, self._goal_cb, queue_size=1
        )

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

    def _goal_cb(self, msg):
        if not (
            math.isfinite(msg.pose.position.x)
            and math.isfinite(msg.pose.position.y)
        ):
            rospy.logerr("Ignoring a non-finite RViz 2D goal.")
            return

        with self._lock:
            state_ready = (
                self._state.connected
                and self._state.armed
                and self._state.mode == "OFFBOARD"
            )
        if not state_ready:
            rospy.logwarn(
                "Ignoring the RViz goal while SITL is not armed in OFFBOARD. "
                "Run ./launch/sim.sh arm first."
            )
            return

        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.frame_id or msg.header.frame_id or "world"
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
