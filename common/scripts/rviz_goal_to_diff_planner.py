#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped


class RvizGoalToDiffPlanner:
    def __init__(self):
        rospy.init_node("rviz_goal_to_diff_planner")

        self.default_z = rospy.get_param("~default_z", 0.8)
        self.input_topic = rospy.get_param(
            "~input_topic", "/sim2real/rviz_goal"
        )
        self.output_topic = rospy.get_param("~output_topic", "/goal")
        self.frame_id_override = rospy.get_param("~frame_id", "")

        self.goal_pub = rospy.Publisher(
            self.output_topic, PoseStamped, queue_size=1, latch=True
        )
        rospy.Subscriber(
            self.input_topic, PoseStamped, self.nav_goal_cb, queue_size=1
        )

        rospy.loginfo(
            "rviz_goal_to_diff_planner started: %s -> %s, "
            "default_z=%.3f",
            self.input_topic,
            self.output_topic,
            self.default_z,
        )

    def resolve_frame(self, header):
        if self.frame_id_override:
            return self.frame_id_override
        if header.frame_id:
            return header.frame_id
        return "world"

    def nav_goal_cb(self, msg):
        out = PoseStamped()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = self.resolve_frame(msg.header)
        out.pose = msg.pose
        out.pose.position.z = self.default_z
        self.goal_pub.publish(out)

        rospy.loginfo(
            "2D Nav Goal -> %s: x=%.3f y=%.3f z=%.3f frame=%s",
            self.output_topic,
            out.pose.position.x,
            out.pose.position.y,
            out.pose.position.z,
            out.header.frame_id,
        )

if __name__ == "__main__":
    try:
        RvizGoalToDiffPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
