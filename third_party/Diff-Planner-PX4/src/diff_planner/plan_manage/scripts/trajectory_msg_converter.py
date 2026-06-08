#!/usr/bin/env python3

"""@trajectory_msg_converter.py
This node converts Fast-Planner reference trajectory message to MultiDOFJointTrajectory which is accepted by geometric_controller
Authors: Mohamed Abdelkader
"""

# Imports
import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint # for geometric_controller
from quadrotor_msgs.msg import PositionCommand # for Fast-Planner
from geometry_msgs.msg import PoseStamped, Transform, Twist
from mavros_msgs.msg import State
from tf.transformations import quaternion_from_euler

class MessageConverter:
    def __init__(self):
        rospy.init_node('trajectory_msg_converter')

        rospy.logwarn("---------------OK!")

        fast_planner_traj_topic = rospy.get_param('~traj_topic', 'planning/pos_cmd')
        traj_pub_topic = rospy.get_param('~traj_pub_topic', 'command/trajectory')
        self.goal_topic = rospy.get_param('~goal_topic', '/goal')
        self.require_offboard = rospy.get_param('~require_offboard', True)

        # Publisher for geometric_controller
        self.traj_pub = rospy.Publisher(traj_pub_topic, MultiDOFJointTrajectory, queue_size=1)
        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1, latch=True)

        self.last_goal = None
        self.current_state = State()
        self.was_offboard_ready = False
        self.last_preoffboard_traj_id = 0

        # Subscriber for Fast-Planner reference trajectory
        rospy.Subscriber(fast_planner_traj_topic, PositionCommand, self.fastPlannerTrajCallback, tcp_nodelay=True)
        rospy.Subscriber(self.goal_topic, PoseStamped, self.goalCallback, queue_size=1)
        rospy.Subscriber('/mavros/state', State, self.stateCallback, queue_size=1)

        rospy.spin()

    def isOffboardReady(self):
        if not self.require_offboard:
            return True
        return self.current_state.connected and self.current_state.armed and self.current_state.mode == 'OFFBOARD'

    def goalCallback(self, msg):
        self.last_goal = msg

    def stateCallback(self, msg):
        self.current_state = msg
        offboard_ready = self.isOffboardReady()

        if offboard_ready and not self.was_offboard_ready:
            rospy.loginfo('[trajectory_msg_converter] OFFBOARD detected.')
            if self.last_goal is not None:
                goal = PoseStamped()
                goal.header = self.last_goal.header
                goal.header.stamp = rospy.Time.now()
                goal.pose = self.last_goal.pose
                rospy.sleep(0.1)
                self.goal_pub.publish(goal)
                rospy.loginfo(
                    '[trajectory_msg_converter] Re-published last RViz goal after OFFBOARD: x=%.3f y=%.3f z=%.3f',
                    goal.pose.position.x,
                    goal.pose.position.y,
                    goal.pose.position.z,
                )
            else:
                rospy.logwarn('[trajectory_msg_converter] OFFBOARD entered, but no RViz goal has been received yet.')

        self.was_offboard_ready = offboard_ready

    def fastPlannerTrajCallback(self, msg):
        if not self.isOffboardReady():
            self.last_preoffboard_traj_id = max(self.last_preoffboard_traj_id, msg.trajectory_id)
            return

        if msg.trajectory_id <= self.last_preoffboard_traj_id:
            rospy.logwarn_throttle(
                1.0,
                '[trajectory_msg_converter] Waiting for fresh post-OFFBOARD trajectory. Ignoring traj_id=%d <= preoffboard=%d',
                msg.trajectory_id,
                self.last_preoffboard_traj_id,
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
        # rospy.logwarn("Publishing OK!")

if __name__ == '__main__':
    obj = MessageConverter()
