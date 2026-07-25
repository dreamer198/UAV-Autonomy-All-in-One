#!/usr/bin/env python3

"""@trajectory_msg_converter.py
This node converts Fast-Planner reference trajectory message to MultiDOFJointTrajectory which is accepted by geometric_controller
Authors: Mohamed Abdelkader
"""

# Imports
import copy
import math
import threading
import time

import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint # for geometric_controller
from quadrotor_msgs.msg import PositionCommand # for Fast-Planner
from geometry_msgs.msg import PoseStamped, Transform, Twist
from mavros_msgs.msg import State
from traj_utils.msg import PolyTraj
from tf.transformations import quaternion_from_euler

class MessageConverter:
    def __init__(self):
        rospy.init_node('trajectory_msg_converter')

        fast_planner_traj_topic = rospy.get_param('~traj_topic', 'planning/pos_cmd')
        poly_traj_topic = rospy.get_param('~poly_traj_topic', '/drone_0_planning/trajectory')
        traj_pub_topic = rospy.get_param('~traj_pub_topic', 'command/trajectory')
        self.goal_topic = rospy.get_param('~goal_topic', '/goal')
        self.require_offboard = rospy.get_param('~require_offboard', True)
        self.replay_cached_goal_on_offboard = rospy.get_param(
            '~replay_cached_goal_on_offboard', True
        )
        self.goal_match_tolerance = float(rospy.get_param('~goal_match_tolerance', 0.05))
        if not math.isfinite(self.goal_match_tolerance) or self.goal_match_tolerance <= 0.0:
            raise ValueError('~goal_match_tolerance must be positive')

        # Publisher for geometric_controller
        self.traj_pub = rospy.Publisher(traj_pub_topic, MultiDOFJointTrajectory, queue_size=1)
        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1, latch=True)

        self.last_goal = None
        self.has_cached_goal_param = rospy.resolve_name('~has_cached_goal')
        rospy.set_param(self.has_cached_goal_param, False)
        self.goal_generation = 0
        self.current_state = State()
        self.was_offboard_ready = False
        self.awaiting_fresh_goal_trajectory = False
        self.fresh_trajectory_id = None
        self.fresh_trajectory_requested_at = rospy.Time(0)
        self.fresh_trajectory_received_at = 0.0
        self.fresh_goal_generation = None
        self.fresh_goal_position = None
        self.fresh_request_epoch = 0
        self.state_lock = threading.Lock()

        # Subscriber for Fast-Planner reference trajectory
        rospy.Subscriber(fast_planner_traj_topic, PositionCommand, self.fastPlannerTrajCallback, tcp_nodelay=True)
        rospy.Subscriber(poly_traj_topic, PolyTraj, self.polyTrajCallback, queue_size=10)
        rospy.Subscriber(self.goal_topic, PoseStamped, self.goalCallback, queue_size=1)
        rospy.Subscriber('/mavros/state', State, self.stateCallback, queue_size=1)

        rospy.spin()

    @staticmethod
    def _stateIsOffboardReady(state):
        return state.connected and state.armed and state.mode == 'OFFBOARD'

    def goalCallback(self, msg):
        incoming_position = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )
        with self.state_lock:
            if (
                self.require_offboard
                and not self.replay_cached_goal_on_offboard
                and not self._stateIsOffboardReady(self.current_state)
            ):
                rospy.logwarn_throttle(
                    1.0,
                    '[trajectory_msg_converter] Ignoring a goal received before armed OFFBOARD.',
                )
                return
            same_fresh_request = (
                self.awaiting_fresh_goal_trajectory
                and self.fresh_goal_position == incoming_position
                and self.fresh_trajectory_requested_at == msg.header.stamp
            )
            self.last_goal = copy.deepcopy(msg)
            self.goal_generation += 1
            if (
                self.awaiting_fresh_goal_trajectory
                and not same_fresh_request
                and (
                    not self.require_offboard
                    or self._stateIsOffboardReady(self.current_state)
                )
            ):
                self.fresh_request_epoch += 1
                self.fresh_trajectory_id = None
                self.fresh_trajectory_received_at = 0.0
                self.fresh_goal_generation = self.goal_generation
                self.fresh_goal_position = incoming_position
                self.fresh_trajectory_requested_at = msg.header.stamp
            rospy.set_param(self.has_cached_goal_param, True)

    def stateCallback(self, msg):
        offboard_ready = not self.require_offboard or self._stateIsOffboardReady(msg)
        goal = None
        transition_epoch = None
        cached_goal_cleared = False

        # Close the forwarding gate before exposing an armed/OFFBOARD state to
        # the high-rate PositionCommand callback.
        with self.state_lock:
            leaving_offboard = (
                self.require_offboard
                and not offboard_ready
                and self.was_offboard_ready
            )
            entering_offboard = (
                self.require_offboard
                and offboard_ready
                and not self.was_offboard_ready
            )
            if entering_offboard:
                self.fresh_request_epoch += 1
                transition_epoch = self.fresh_request_epoch
                self.awaiting_fresh_goal_trajectory = True
                self.fresh_trajectory_id = None
                self.fresh_trajectory_received_at = 0.0
                if self.replay_cached_goal_on_offboard and self.last_goal is not None:
                    goal = copy.deepcopy(self.last_goal)
                    self.fresh_goal_generation = self.goal_generation
                    self.fresh_goal_position = (
                        goal.pose.position.x,
                        goal.pose.position.y,
                        goal.pose.position.z,
                    )
                else:
                    if not self.replay_cached_goal_on_offboard:
                        self.last_goal = None
                        cached_goal_cleared = True
                    self.fresh_goal_generation = None
                    self.fresh_goal_position = None
            elif leaving_offboard:
                self.fresh_request_epoch += 1
                self.awaiting_fresh_goal_trajectory = False
                self.fresh_trajectory_id = None
                self.fresh_trajectory_received_at = 0.0
                if not self.replay_cached_goal_on_offboard:
                    self.last_goal = None
                    self.fresh_goal_generation = None
                    self.fresh_goal_position = None
                    cached_goal_cleared = True
            self.current_state = msg
            self.was_offboard_ready = offboard_ready
            if cached_goal_cleared:
                rospy.set_param(self.has_cached_goal_param, False)

        if entering_offboard:
            rospy.loginfo('[trajectory_msg_converter] OFFBOARD detected.')
            if goal is not None:
                goal.header.stamp = rospy.Time.now()
                published = False
                with self.state_lock:
                    if (
                        transition_epoch == self.fresh_request_epoch
                        and self.awaiting_fresh_goal_trajectory
                        and self._stateIsOffboardReady(self.current_state)
                    ):
                        self.fresh_trajectory_requested_at = goal.header.stamp
                        self.goal_pub.publish(goal)
                        published = True
                if published:
                    rospy.loginfo(
                        '[trajectory_msg_converter] Re-published last RViz goal after OFFBOARD: x=%.3f y=%.3f z=%.3f',
                        goal.pose.position.x,
                        goal.pose.position.y,
                        goal.pose.position.z,
                    )
                else:
                    rospy.loginfo('[trajectory_msg_converter] A newer goal superseded the cached OFFBOARD replan request.')
            else:
                rospy.logwarn('[trajectory_msg_converter] OFFBOARD entered without an RViz goal; trajectory forwarding remains blocked.')

    def polyTrajCallback(self, msg):
        if not msg.armable:
            return
        with self.state_lock:
            if (
                self.require_offboard
                and not self._stateIsOffboardReady(self.current_state)
            ) or not self.awaiting_fresh_goal_trajectory:
                return
            if self.last_goal is None:
                return
            if self.fresh_goal_position is None:
                return
            goal_generation = self.fresh_goal_generation
            goal_position = self.fresh_goal_position
            requested_at = self.fresh_trajectory_requested_at
            request_epoch = self.fresh_request_epoch

        if msg.goal_stamp != requested_at:
            rospy.logwarn_throttle(
                1.0,
                '[trajectory_msg_converter] Ignoring a trajectory for a different goal request.',
            )
            return

        if msg.traj_id <= 0:
            return

        trajectory_goal = tuple(float(value) for value in msg.goal_position)
        if not all(math.isfinite(value) for value in trajectory_goal):
            return

        dx = trajectory_goal[0] - goal_position[0]
        dy = trajectory_goal[1] - goal_position[1]
        dz = trajectory_goal[2] - goal_position[2]
        if (dx * dx + dy * dy + dz * dz) ** 0.5 > self.goal_match_tolerance:
            rospy.logwarn_throttle(
                1.0,
                '[trajectory_msg_converter] Ignoring post-OFFBOARD trajectory for a different goal.',
            )
            return

        with self.state_lock:
            if (
                not self.awaiting_fresh_goal_trajectory
                or goal_generation != self.fresh_goal_generation
                or request_epoch != self.fresh_request_epoch
            ):
                return
            self.fresh_trajectory_id = msg.traj_id
            self.fresh_trajectory_received_at = time.monotonic()
        rospy.loginfo(
            '[trajectory_msg_converter] Fresh post-OFFBOARD plan confirmed: traj_id=%d goal=(%.3f, %.3f, %.3f)',
            msg.traj_id,
            msg.goal_position[0],
            msg.goal_position[1],
            msg.goal_position[2],
        )

    def fastPlannerTrajCallback(self, msg):
        command_values = (
            msg.position.x,
            msg.position.y,
            msg.position.z,
            msg.velocity.x,
            msg.velocity.y,
            msg.velocity.z,
            msg.acceleration.x,
            msg.acceleration.y,
            msg.acceleration.z,
            msg.yaw,
            msg.yaw_dot,
        )
        command_valid = (
            msg.trajectory_id > 0
            and msg.trajectory_flag
            == PositionCommand.TRAJECTORY_STATUS_READY
            and all(math.isfinite(value) for value in command_values)
        )
        if not command_valid:
            rospy.logwarn_throttle(
                1.0,
                '[trajectory_msg_converter] Ignoring an invalid/non-finite trajectory command (id=%d).',
                msg.trajectory_id,
            )
            return

        with self.state_lock:
            offboard_ready = (
                not self.require_offboard
                or self._stateIsOffboardReady(self.current_state)
            )
            if not offboard_ready:
                return
            awaiting_fresh = self.awaiting_fresh_goal_trajectory
            fresh_ready = (
                self.fresh_trajectory_id is not None
                and msg.trajectory_id == self.fresh_trajectory_id
                and time.monotonic() >= self.fresh_trajectory_received_at
            )
            if awaiting_fresh and fresh_ready:
                self.awaiting_fresh_goal_trajectory = False

        if awaiting_fresh:
            if fresh_ready:
                rospy.loginfo(
                    '[trajectory_msg_converter] Forwarding fresh post-OFFBOARD trajectory id=%d.',
                    msg.trajectory_id,
                )
            else:
                rospy.logwarn_throttle(
                    1.0,
                    '[trajectory_msg_converter] Waiting for a fresh post-OFFBOARD plan for the active goal.',
                )
                return

        # position and yaw
        pose = Transform()
        pose.translation.x = msg.position.x
        pose.translation.y = msg.position.y
        pose.translation.z = msg.position.z
        q = quaternion_from_euler(0, 0, msg.yaw) # RPY
        pose.rotation.x = q[0]
        pose.rotation.y = q[1]
        pose.rotation.z = q[2]
        pose.rotation.w = q[3]

        # velocity
        vel = Twist()
        vel.linear = msg.velocity
        vel.angular.z = msg.yaw_dot

        # acceleration
        acc = Twist()
        acc.linear = msg.acceleration

        traj_point = MultiDOFJointTrajectoryPoint()
        traj_point.transforms.append(pose)
        traj_point.velocities.append(vel)
        traj_point.accelerations.append(acc)

        traj_msg = MultiDOFJointTrajectory()

        traj_msg.header = msg.header
        traj_msg.points.append(traj_point)
        self.traj_pub.publish(traj_msg)

if __name__ == '__main__':
    obj = MessageConverter()
