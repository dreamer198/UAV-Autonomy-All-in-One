#!/usr/bin/env python3
import math
import os
import threading

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def env_float(name, default):
    value = os.environ.get(name)
    if value is None or value == "":
        return float(default)
    return float(value)


def quat_to_rot(qx, qy, qz, qw):
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-12:
        return [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]

    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return [
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ]


def rot_to_quat(r):
    t = r[0][0] + r[1][1] + r[2][2]
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r[2][1] - r[1][2]) / s
        qy = (r[0][2] - r[2][0]) / s
        qz = (r[1][0] - r[0][1]) / s
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2.0
        qw = (r[2][1] - r[1][2]) / s
        qx = 0.25 * s
        qy = (r[0][1] + r[1][0]) / s
        qz = (r[0][2] + r[2][0]) / s
    elif r[1][1] > r[2][2]:
        s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2.0
        qw = (r[0][2] - r[2][0]) / s
        qx = (r[0][1] + r[1][0]) / s
        qy = 0.25 * s
        qz = (r[1][2] + r[2][1]) / s
    else:
        s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2.0
        qw = (r[1][0] - r[0][1]) / s
        qx = (r[0][2] + r[2][0]) / s
        qy = (r[1][2] + r[2][1]) / s
        qz = 0.25 * s
    return qx, qy, qz, qw


def rpy_to_rot(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = [
        [1.0, 0.0, 0.0],
        [0.0, cr, -sr],
        [0.0, sr, cr],
    ]
    ry = [
        [cp, 0.0, sp],
        [0.0, 1.0, 0.0],
        [-sp, 0.0, cp],
    ]
    rz = [
        [cy, -sy, 0.0],
        [sy, cy, 0.0],
        [0.0, 0.0, 1.0],
    ]
    return matmul(matmul(rz, ry), rx)


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def matvec(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def transpose(a):
    return [[a[j][i] for j in range(3)] for i in range(3)]


def skew(v):
    return [
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ]


def scale_matrix(a, scale):
    return [[scale * value for value in row] for row in a]


def block_matrix(a, b, c, d):
    return [a[i] + b[i] for i in range(3)] + [c[i] + d[i] for i in range(3)]


def matmul_n(a, b):
    return [
        [sum(a[i][k] * b[k][j] for k in range(len(b))) for j in range(len(b[0]))]
        for i in range(len(a))
    ]


def transpose_n(a):
    return [[a[j][i] for j in range(len(a))] for i in range(len(a[0]))]


def transform_covariance(covariance, jacobian):
    if len(covariance) != 36:
        return covariance
    matrix = [list(covariance[row * 6 : (row + 1) * 6]) for row in range(6)]
    transformed = matmul_n(matmul_n(jacobian, matrix), transpose_n(jacobian))
    return [transformed[row][column] for row in range(6) for column in range(6)]


def add(a, b):
    return [a[i] + b[i] for i in range(3)]


def neg(a):
    return [-a[i] for i in range(3)]


def cross(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


class OdomToBase:
    def __init__(self):
        input_topic = rospy.get_param("~input_topic", os.environ.get("ODOM_RAW_TOPIC", "/Odometry"))
        output_topic = rospy.get_param(
            "~output_topic",
            os.environ.get("ODOM_BASE_TOPIC", "/localization/odom"),
        )
        self.output_frame_id = rospy.get_param("~output_frame_id", "world")
        self.output_child_frame_id = rospy.get_param("~output_child_frame_id", "base_link")

        imu_topic = rospy.get_param("~imu_topic", os.environ.get("IMU_TOPIC", "/livox/imu"))
        self.imu_timeout = float(rospy.get_param("~imu_timeout", 0.1))

        # T_base_fastlio_body: FAST-LIO's child frame is the MID-360 internal IMU,
        # not the LiDAR optical/point-cloud origin.
        mx = rospy.get_param("~mount_x", env_float("MOUNT_X", 0.109))
        my = rospy.get_param("~mount_y", env_float("MOUNT_Y", 0.024))
        mz = rospy.get_param("~mount_z", env_float("MOUNT_Z", 0.006))
        roll_deg = rospy.get_param("~mount_roll_deg", env_float("MOUNT_ROLL_DEG", 0.0))
        pitch_deg = rospy.get_param("~mount_pitch_deg", env_float("MOUNT_PITCH_DEG", 34.9))
        yaw_deg = rospy.get_param("~mount_yaw_deg", env_float("MOUNT_YAW_DEG", 0.5))

        self.r_base_sensor = rpy_to_rot(
            math.radians(roll_deg),
            math.radians(pitch_deg),
            math.radians(yaw_deg),
        )
        self.t_base_sensor = [float(mx), float(my), float(mz)]
        self.r_sensor_base = transpose(self.r_base_sensor)
        self.t_sensor_base = matvec(self.r_sensor_base, neg(self.t_base_sensor))

        self.imu_lock = threading.Lock()
        self.latest_imu = None
        self.pub = rospy.Publisher(output_topic, Odometry, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.sub = rospy.Subscriber(input_topic, Odometry, self.cb, queue_size=20)
        self.imu_sub = rospy.Subscriber(imu_topic, Imu, self.imu_cb, queue_size=100)

        rospy.loginfo("odom_to_base input_topic: %s", input_topic)
        rospy.loginfo("odom_to_base output_topic: %s", output_topic)
        rospy.loginfo("odom_to_base angular velocity source: %s (timeout %.3f s)", imu_topic, self.imu_timeout)
        rospy.loginfo(
            "T_base_fastlio_body xyz=[%.3f, %.3f, %.3f] rpy_deg=[%.2f, %.2f, %.2f]",
            mx,
            my,
            mz,
            roll_deg,
            pitch_deg,
            yaw_deg,
        )

    def imu_cb(self, msg):
        stamp = msg.header.stamp
        if stamp == rospy.Time():
            stamp = rospy.Time.now()
        angular_covariance = list(msg.angular_velocity_covariance)
        if angular_covariance and angular_covariance[0] < 0.0:
            angular_covariance = None
        sample = (
            stamp,
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            angular_covariance,
        )
        with self.imu_lock:
            self.latest_imu = sample

    def angular_velocity_for(self, stamp, fallback):
        with self.imu_lock:
            sample = self.latest_imu
        if sample is None:
            return fallback, None
        sample_stamp, angular_velocity, angular_covariance = sample
        if abs((stamp - sample_stamp).to_sec()) > self.imu_timeout:
            rospy.logwarn_throttle(
                10.0,
                "odom_to_base has no time-aligned IMU sample; using odometry angular velocity",
            )
            return fallback, None
        return angular_velocity, angular_covariance

    def cb(self, msg):
        out = Odometry()
        out.header = msg.header
        if self.output_frame_id:
            out.header.frame_id = self.output_frame_id
        out.child_frame_id = self.output_child_frame_id

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        p_world_sensor = [p.x, p.y, p.z]
        r_world_sensor = quat_to_rot(q.x, q.y, q.z, q.w)

        r_world_base = matmul(r_world_sensor, self.r_sensor_base)
        p_world_base = add(p_world_sensor, matvec(r_world_sensor, self.t_sensor_base))
        qx, qy, qz, qw = rot_to_quat(r_world_base)

        out.pose.pose.position.x = p_world_base[0]
        out.pose.pose.position.y = p_world_base[1]
        out.pose.pose.position.z = p_world_base[2]
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw
        lever_world = matvec(r_world_sensor, self.t_sensor_base)
        identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        zero = [[0.0, 0.0, 0.0] for _ in range(3)]
        pose_jacobian = block_matrix(identity, scale_matrix(skew(lever_world), -1.0), zero, identity)
        out.pose.covariance = transform_covariance(msg.pose.covariance, pose_jacobian)

        v_sensor = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ]
        w_odom = [
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
        ]
        w_sensor, imu_angular_covariance = self.angular_velocity_for(msg.header.stamp, w_odom)
        v_base_in_sensor = add(v_sensor, cross(w_sensor, self.t_sensor_base))
        v_base = matvec(self.r_base_sensor, v_base_in_sensor)
        w_base = matvec(self.r_base_sensor, w_sensor)

        out.twist.twist.linear.x = v_base[0]
        out.twist.twist.linear.y = v_base[1]
        out.twist.twist.linear.z = v_base[2]
        out.twist.twist.angular.x = w_base[0]
        out.twist.twist.angular.y = w_base[1]
        out.twist.twist.angular.z = w_base[2]
        twist_covariance = list(msg.twist.covariance)
        if imu_angular_covariance is not None:
            for row in range(3):
                for column in range(3):
                    twist_covariance[(row + 3) * 6 + column + 3] = imu_angular_covariance[row * 3 + column]
        twist_jacobian = block_matrix(
            self.r_base_sensor,
            scale_matrix(matmul(self.r_base_sensor, skew(self.t_sensor_base)), -1.0),
            zero,
            self.r_base_sensor,
        )
        out.twist.covariance = transform_covariance(twist_covariance, twist_jacobian)

        self.pub.publish(out)

        transform = TransformStamped()
        transform.header = out.header
        transform.child_frame_id = out.child_frame_id
        transform.transform.translation.x = out.pose.pose.position.x
        transform.transform.translation.y = out.pose.pose.position.y
        transform.transform.translation.z = out.pose.pose.position.z
        transform.transform.rotation = out.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


if __name__ == "__main__":
    rospy.init_node("odom_to_base")
    OdomToBase()
    rospy.spin()
