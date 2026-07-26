#!/usr/bin/env python3

import importlib.util
import os
import sys
import types
import unittest
from unittest import mock


def load_module():
    rospy = types.ModuleType("rospy")
    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.PoseStamped = object
    nav_msgs = types.ModuleType("nav_msgs")
    nav_msgs_msg = types.ModuleType("nav_msgs.msg")
    nav_msgs_msg.Odometry = object
    modules = {
        "rospy": rospy,
        "geometry_msgs": geometry_msgs,
        "geometry_msgs.msg": geometry_msgs_msg,
        "nav_msgs": nav_msgs,
        "nav_msgs.msg": nav_msgs_msg,
    }
    path = os.path.join(
        os.path.dirname(__file__), "..", "scripts", "odom_to_pose.py"
    )
    with mock.patch.dict(sys.modules, modules):
        spec = importlib.util.spec_from_file_location(
            "tested_odom_to_pose", path
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    return module


class OdomToPoseTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_module()

    def test_rejects_stale_future_and_zero_measurement_timestamps(self):
        reason = self.module.timestamp_age_reason
        self.assertEqual(reason(10.0, 10.1, 0.2, 0.05), "")
        self.assertIn("old", reason(9.0, 10.0, 0.2, 0.05))
        self.assertIn("future", reason(10.1, 10.0, 0.2, 0.05))
        self.assertIn("zero", reason(0.0, 10.0, 0.2, 0.05))

    def test_bridge_preserves_stamp_and_does_not_repeat_a_measurement(self):
        path = os.path.join(
            os.path.dirname(__file__), "..", "scripts", "odom_to_pose.py"
        )
        with open(path, "r", encoding="utf-8") as stream:
            source = stream.read()
        self.assertIn("pose.header.stamp = header_snapshot.stamp", source)
        self.assertIn("latest_stamp == self.last_published_stamp", source)
        self.assertNotIn("pose.header.stamp = now", source)

    def test_cloud_relay_transforms_instead_of_relabelling_input(self):
        path = os.path.join(
            os.path.dirname(__file__), "..", "scripts", "cloud_relay.py"
        )
        with open(path, "r", encoding="utf-8") as stream:
            source = stream.read()
        self.assertIn("do_transform_cloud(msg, transform)", source)
        self.assertNotIn("msg.header.frame_id =", source)


if __name__ == "__main__":
    unittest.main()
