#!/usr/bin/env python3

import math
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseStamped, Transform, Twist
from mavros_msgs.msg import AttitudeTarget, State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from trajectory_msgs.msg import (
    MultiDOFJointTrajectory,
    MultiDOFJointTrajectoryPoint,
)


def yaw_quaternion(yaw_degrees):
    half = math.radians(yaw_degrees) / 2.0
    return (0.0, 0.0, math.sin(half), math.cos(half))


def quaternion_yaw_degrees(quaternion):
    return math.degrees(
        math.atan2(
            2.0
            * (
                quaternion.w * quaternion.z
                + quaternion.x * quaternion.y
            ),
            1.0
            - 2.0
            * (
                quaternion.y * quaternion.y
                + quaternion.z * quaternion.z
            ),
        )
    )


def angular_error_degrees(first, second):
    return abs((first - second + 180.0) % 360.0 - 180.0)


class Se3HandoffBehaviorTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.mode = "AUTO.TAKEOFF"
        self.publish_trajectory = False
        self.running = True
        self.position_messages = []
        self.attitude_messages = []

        self.state_pub = rospy.Publisher("/mavros/state", State, queue_size=10)
        self.world_odom_pub = rospy.Publisher(
            "/localization/odom", Odometry, queue_size=10
        )
        self.local_odom_pub = rospy.Publisher(
            "/mavros/local_position/odom", Odometry, queue_size=10
        )
        self.imu_pub = rospy.Publisher("/mavros/imu/data", Imu, queue_size=10)
        self.trajectory_pub = rospy.Publisher(
            "/command/trajectory", MultiDOFJointTrajectory, queue_size=10
        )
        rospy.Subscriber(
            "/mavros/setpoint_position/local",
            PoseStamped,
            self._position_callback,
            queue_size=100,
        )
        rospy.Subscriber(
            "/mavros/setpoint_raw/attitude",
            AttitudeTarget,
            self._attitude_callback,
            queue_size=100,
        )
        self.publisher_thread = threading.Thread(target=self._publish_inputs)
        self.publisher_thread.daemon = True
        self.publisher_thread.start()

    def tearDown(self):
        self.running = False
        self.publisher_thread.join(timeout=2.0)

    def _position_callback(self, message):
        with self.lock:
            self.position_messages.append((time.monotonic(), message))

    def _attitude_callback(self, message):
        with self.lock:
            self.attitude_messages.append((time.monotonic(), message))

    @staticmethod
    def _set_quaternion(target, values):
        target.x, target.y, target.z, target.w = values

    def _publish_inputs(self):
        rate = rospy.Rate(100)
        while self.running and not rospy.is_shutdown():
            now = rospy.Time.now()
            state = State()
            state.header.stamp = now
            state.connected = True
            state.armed = True
            with self.lock:
                state.mode = self.mode
                publish_trajectory = self.publish_trajectory
            self.state_pub.publish(state)

            world_odom = Odometry()
            world_odom.header.stamp = now
            world_odom.header.frame_id = "world"
            world_odom.child_frame_id = "base_link"
            self._set_quaternion(
                world_odom.pose.pose.orientation, yaw_quaternion(-45.0)
            )
            self.world_odom_pub.publish(world_odom)

            local_odom = Odometry()
            local_odom.header.stamp = now
            local_odom.header.frame_id = "map"
            local_odom.child_frame_id = "base_link"
            local_odom.pose.pose.position.x = 4.0
            local_odom.pose.pose.position.y = -2.0
            local_odom.pose.pose.position.z = 1.0
            self._set_quaternion(
                local_odom.pose.pose.orientation, yaw_quaternion(45.0)
            )
            self.local_odom_pub.publish(local_odom)

            imu = Imu()
            imu.header.stamp = now
            self._set_quaternion(imu.orientation, yaw_quaternion(45.0))
            imu.linear_acceleration.z = 9.81
            self.imu_pub.publish(imu)

            if publish_trajectory:
                trajectory = MultiDOFJointTrajectory()
                trajectory.header.stamp = now
                trajectory.header.frame_id = "world"
                point = MultiDOFJointTrajectoryPoint()
                transform = Transform()
                self._set_quaternion(transform.rotation, yaw_quaternion(0.0))
                point.transforms.append(transform)
                point.velocities.append(Twist())
                point.accelerations.append(Twist())
                trajectory.points.append(point)
                self.trajectory_pub.publish(trajectory)
            rate.sleep()

    def _wait_until(self, predicate, timeout=4.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if predicate():
                return
            time.sleep(0.01)
        self.fail("condition was not satisfied within {:.1f}s".format(timeout))

    def test_takeoff_hold_and_first_trajectory_are_bumpless(self):
        self._wait_until(lambda: len(self.position_messages) >= 20)
        with self.lock:
            self.assertEqual(len(self.attitude_messages), 0)
            hold = self.position_messages[-1][1]
        self.assertAlmostEqual(hold.pose.position.x, 4.0, places=6)
        self.assertAlmostEqual(hold.pose.position.y, -2.0, places=6)
        self.assertAlmostEqual(hold.pose.position.z, 1.0, places=6)
        self.assertLess(
            angular_error_degrees(
                quaternion_yaw_degrees(hold.pose.orientation), 45.0
            ),
            0.1,
        )

        with self.lock:
            before_offboard_positions = len(self.position_messages)
            self.mode = "OFFBOARD"
        time.sleep(0.35)
        with self.lock:
            self.assertGreater(
                len(self.position_messages), before_offboard_positions + 20
            )
            self.assertEqual(len(self.attitude_messages), 0)
            self.publish_trajectory = True

        self._wait_until(lambda: len(self.attitude_messages) >= 10)
        with self.lock:
            first_attitude = self.attitude_messages[0][1]
        self.assertLess(
            angular_error_degrees(
                quaternion_yaw_degrees(first_attitude.orientation), 45.0
            ),
            5.0,
        )
        self.assertAlmostEqual(first_attitude.thrust, 0.5, delta=0.03)

        time.sleep(0.7)
        with self.lock:
            last_attitude = self.attitude_messages[-1][1]
            self.publish_trajectory = False
        self.assertLess(
            angular_error_degrees(
                quaternion_yaw_degrees(last_attitude.orientation), 90.0
            ),
            5.0,
        )

        time.sleep(0.3)
        with self.lock:
            last_attitude_age = (
                time.monotonic() - self.attitude_messages[-1][0]
            )
            last_position_age = (
                time.monotonic() - self.position_messages[-1][0]
            )
        self.assertGreater(last_attitude_age, 0.15)
        self.assertLess(last_position_age, 0.08)


if __name__ == "__main__":
    rospy.init_node("se3_handoff_behavior")
    rostest.rosrun(
        "se3_controller",
        "se3_handoff_behavior",
        Se3HandoffBehaviorTest,
    )
