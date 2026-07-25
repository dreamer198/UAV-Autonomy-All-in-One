#!/usr/bin/env python3

import struct

import rospy
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import PointCloud2, PointField


FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name="tag", offset=16, datatype=PointField.UINT8, count=1),
    PointField(name="line", offset=17, datatype=PointField.UINT8, count=1),
]
POINT_STRUCT = struct.Struct("<ffffBB")


def custom_msg_to_pointcloud2(msg, point_stride=1, frame_id=""):
    """Convert a Livox CustomMsg without changing the point coordinates."""
    selected_points = msg.points[::point_stride]
    data = bytearray(len(selected_points) * POINT_STRUCT.size)

    for index, point in enumerate(selected_points):
        POINT_STRUCT.pack_into(
            data,
            index * POINT_STRUCT.size,
            point.x,
            point.y,
            point.z,
            float(point.reflectivity),
            point.tag,
            point.line,
        )

    cloud = PointCloud2()
    cloud.header = msg.header
    if frame_id:
        cloud.header.frame_id = frame_id
    cloud.height = 1
    cloud.width = len(selected_points)
    cloud.fields = FIELDS
    cloud.is_bigendian = False
    cloud.point_step = POINT_STRUCT.size
    cloud.row_step = cloud.point_step * cloud.width
    cloud.data = bytes(data)
    cloud.is_dense = False
    return cloud


class LivoxCustomToPointCloud2:
    def __init__(self):
        input_topic = rospy.get_param("~input_topic", "/livox/lidar")
        output_topic = rospy.get_param("~output_topic", "/livox/lidar_points")
        self.frame_id = rospy.get_param("~frame_id", "")
        self.point_stride = int(rospy.get_param("~point_stride", 1))
        if self.point_stride < 1:
            raise ValueError("~point_stride must be at least 1")

        self.publisher = rospy.Publisher(output_topic, PointCloud2, queue_size=1)
        self.subscriber = rospy.Subscriber(
            input_topic,
            CustomMsg,
            self.callback,
            queue_size=1,
            buff_size=16 * 1024 * 1024,
            tcp_nodelay=True,
        )
        rospy.loginfo(
            "livox_custom_to_pointcloud2: %s -> %s (frame=%s, stride=%d)",
            input_topic,
            output_topic,
            self.frame_id or "preserve",
            self.point_stride,
        )

    def callback(self, msg):
        if msg.point_num != len(msg.points):
            rospy.logwarn_throttle(
                5.0,
                "Livox point_num=%d differs from points length=%d",
                msg.point_num,
                len(msg.points),
            )
        self.publisher.publish(
            custom_msg_to_pointcloud2(msg, self.point_stride, self.frame_id)
        )


if __name__ == "__main__":
    rospy.init_node("livox_custom_to_pointcloud2")
    LivoxCustomToPointCloud2()
    rospy.spin()
