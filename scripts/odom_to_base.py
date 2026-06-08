#!/usr/bin/env python3
import math
import os

import rospy
from nav_msgs.msg import Odometry


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
        output_topic = rospy.get_param("~output_topic", os.environ.get("ODOM_BASE_TOPIC", "/Odometry_base"))
        self.output_frame_id = rospy.get_param("~output_frame_id", "")
        self.output_child_frame_id = rospy.get_param("~output_child_frame_id", "base_link")

        mx = rospy.get_param("~mount_x", env_float("MOUNT_X", 0.0))
        my = rospy.get_param("~mount_y", env_float("MOUNT_Y", 0.0))
        mz = rospy.get_param("~mount_z", env_float("MOUNT_Z", 0.10))
        roll_deg = rospy.get_param("~mount_roll_deg", env_float("MOUNT_ROLL_DEG", 0.0))
        pitch_deg = rospy.get_param("~mount_pitch_deg", env_float("MOUNT_PITCH_DEG", 30.0))
        yaw_deg = rospy.get_param("~mount_yaw_deg", env_float("MOUNT_YAW_DEG", 0.0))

        self.r_base_sensor = rpy_to_rot(
            math.radians(roll_deg),
            math.radians(pitch_deg),
            math.radians(yaw_deg),
        )
        self.t_base_sensor = [float(mx), float(my), float(mz)]
        self.r_sensor_base = transpose(self.r_base_sensor)
        self.t_sensor_base = matvec(self.r_sensor_base, neg(self.t_base_sensor))

        self.pub = rospy.Publisher(output_topic, Odometry, queue_size=10)
        self.sub = rospy.Subscriber(input_topic, Odometry, self.cb, queue_size=20)

        rospy.loginfo("odom_to_base input_topic: %s", input_topic)
        rospy.loginfo("odom_to_base output_topic: %s", output_topic)
        rospy.loginfo(
            "T_base_sensor xyz=[%.3f, %.3f, %.3f] rpy_deg=[%.2f, %.2f, %.2f]",
            mx,
            my,
            mz,
            roll_deg,
            pitch_deg,
            yaw_deg,
        )

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
        out.pose.covariance = msg.pose.covariance

        v_sensor = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ]
        w_sensor = [
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
        ]
        v_base_in_sensor = add(v_sensor, cross(w_sensor, self.t_sensor_base))
        v_base = matvec(self.r_base_sensor, v_base_in_sensor)
        w_base = matvec(self.r_base_sensor, w_sensor)

        out.twist.twist.linear.x = v_base[0]
        out.twist.twist.linear.y = v_base[1]
        out.twist.twist.linear.z = v_base[2]
        out.twist.twist.angular.x = w_base[0]
        out.twist.twist.angular.y = w_base[1]
        out.twist.twist.angular.z = w_base[2]
        out.twist.covariance = msg.twist.covariance

        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("odom_to_base")
    OdomToBase()
    rospy.spin()
