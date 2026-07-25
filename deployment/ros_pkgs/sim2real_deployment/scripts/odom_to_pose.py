#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class OdomToPose:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.publish_rate = float(rospy.get_param("~publish_rate", 30.0))
        self.max_input_age = rospy.Duration.from_sec(float(rospy.get_param("~max_input_age", 0.2)))
        self.use_input_stamp = bool(rospy.get_param("~use_input_stamp", False))
        self.use_input_frame_id = bool(rospy.get_param("~use_input_frame_id", False))

        odom_topic = rospy.get_param("~odom_topic", "/localization/odom")
        pose_topic = rospy.get_param("~pose_topic", "/mavros/vision_pose/pose")

        if self.publish_rate <= 0.0:
            raise ValueError("~publish_rate must be greater than 0")

        self.latest_pose = None
        self.latest_header = None
        self.latest_input_time = None

        self.pub = rospy.Publisher(pose_topic, PoseStamped, queue_size=10)
        rospy.Subscriber(odom_topic, Odometry, self.cb, queue_size=20)
        rospy.Timer(rospy.Duration.from_sec(1.0 / self.publish_rate), self.timer_cb)

        rospy.loginfo("odom_to_pose odom_topic: %s", odom_topic)
        rospy.loginfo("odom_to_pose pose_topic: %s", pose_topic)
        rospy.loginfo("odom_to_pose publish_rate: %.1f Hz", self.publish_rate)
        rospy.loginfo("odom_to_pose use_input_stamp: %s", self.use_input_stamp)

    def cb(self, msg):
        self.latest_header = msg.header
        self.latest_pose = msg.pose.pose
        self.latest_input_time = rospy.Time.now()

    def timer_cb(self, _event):
        if self.latest_pose is None or self.latest_header is None or self.latest_input_time is None:
            return

        now = rospy.Time.now()
        age = now - self.latest_input_time
        if age > self.max_input_age:
            rospy.logwarn_throttle(
                1.0,
                "odom_to_pose input timeout: %.3fs",
                age.to_sec(),
            )
            return

        pose = PoseStamped()
        pose.header.stamp = self.latest_header.stamp if self.use_input_stamp else now
        if self.frame_id:
            pose.header.frame_id = self.frame_id
        elif self.use_input_frame_id:
            pose.header.frame_id = self.latest_header.frame_id
        pose.pose = self.latest_pose
        self.pub.publish(pose)


if __name__ == "__main__":
    rospy.init_node("odom_to_pose")
    OdomToPose()
    rospy.spin()
