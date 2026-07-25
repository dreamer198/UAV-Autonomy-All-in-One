#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2


class CloudRelay:
    def __init__(self):
        input_topic = rospy.get_param("~input_topic", "/cloud_registered")
        output_topic = rospy.get_param(
            "~output_topic", "/localization/cloud_registered"
        )
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.publisher = rospy.Publisher(output_topic, PointCloud2, queue_size=1)
        self.subscriber = rospy.Subscriber(
            input_topic, PointCloud2, self.callback, queue_size=1
        )
        rospy.loginfo(
            "cloud_relay: %s -> %s (frame_id=%s)",
            input_topic,
            output_topic,
            self.frame_id or "preserve",
        )

    def callback(self, msg):
        if self.frame_id and msg.header.frame_id != self.frame_id:
            msg.header.frame_id = self.frame_id
        self.publisher.publish(msg)


if __name__ == "__main__":
    rospy.init_node("cloud_relay")
    CloudRelay()
    rospy.spin()
