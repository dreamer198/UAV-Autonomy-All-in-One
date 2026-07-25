#!/usr/bin/env python3

import copy

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class SimulationOdometryAdapter:
    """Expose PX4 SITL odometry through the shared localization contract."""

    def __init__(self):
        input_topic = rospy.get_param(
            "~input_topic", "/mavros/local_position/odom"
        )
        output_topic = rospy.get_param(
            "~output_topic", "/localization/odom"
        )
        self.frame_id = rospy.get_param("~frame_id", "world")
        self.child_frame_id = rospy.get_param("~child_frame_id", "base_link")
        self.publish_tf = bool(rospy.get_param("~publish_tf", True))

        self.publisher = rospy.Publisher(output_topic, Odometry, queue_size=20)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.subscriber = rospy.Subscriber(
            input_topic, Odometry, self.callback, queue_size=20
        )
        rospy.loginfo(
            "simulation odometry adapter: %s -> %s (%s -> %s)",
            input_topic,
            output_topic,
            self.frame_id,
            self.child_frame_id,
        )

    def callback(self, msg):
        output = copy.deepcopy(msg)
        output.header.frame_id = self.frame_id
        output.child_frame_id = self.child_frame_id
        self.publisher.publish(output)

        if not self.publish_tf:
            return
        transform = TransformStamped()
        transform.header = output.header
        transform.child_frame_id = self.child_frame_id
        transform.transform.translation.x = output.pose.pose.position.x
        transform.transform.translation.y = output.pose.pose.position.y
        transform.transform.translation.z = output.pose.pose.position.z
        transform.transform.rotation = output.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


if __name__ == "__main__":
    rospy.init_node("sim_odometry_adapter")
    SimulationOdometryAdapter()
    rospy.spin()
