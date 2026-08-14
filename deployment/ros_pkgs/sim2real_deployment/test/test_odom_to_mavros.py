#!/usr/bin/env python3

import importlib.util
import math
import os
import sys
import types
import unittest
from unittest import mock


def load_module():
    rospy = types.ModuleType("rospy")
    nav_msgs = types.ModuleType("nav_msgs")
    nav_msgs_msg = types.ModuleType("nav_msgs.msg")
    nav_msgs_msg.Odometry = object
    modules = {
        "rospy": rospy,
        "nav_msgs": nav_msgs,
        "nav_msgs.msg": nav_msgs_msg,
    }
    path = os.path.join(
        os.path.dirname(__file__), "..", "scripts", "odom_to_mavros.py"
    )
    with mock.patch.dict(sys.modules, modules):
        spec = importlib.util.spec_from_file_location("tested_odom_to_mavros", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    return module


def vector(x=0.0, y=0.0, z=0.0):
    return types.SimpleNamespace(x=x, y=y, z=z)


def message(stamp=10.0, frame="world", child="base_link"):
    return types.SimpleNamespace(
        header=types.SimpleNamespace(
            frame_id=frame,
            stamp=types.SimpleNamespace(to_sec=lambda: stamp),
        ),
        child_frame_id=child,
        pose=types.SimpleNamespace(
            pose=types.SimpleNamespace(
                position=vector(1.0, 2.0, 3.0),
                orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        ),
        twist=types.SimpleNamespace(
            twist=types.SimpleNamespace(linear=vector(), angular=vector())
        ),
    )


class OdomToMavrosTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_module()

    def reason(self, msg):
        return self.module.odometry_rejection_reason(
            msg, 10.1, 0.2, 0.05, "world", "base_link"
        )

    def test_accepts_fresh_finite_world_to_base_link_odometry(self):
        self.assertEqual(self.reason(message()), "")

    def test_rejects_wrong_parent_or_child_frame(self):
        self.assertIn("parent frame", self.reason(message(frame="map")))
        self.assertIn("child frame", self.reason(message(child="body")))

    def test_rejects_stale_future_and_nonfinite_measurements(self):
        self.assertIn("old", self.reason(message(stamp=9.0)))
        self.assertIn("future", self.reason(message(stamp=10.2)))
        invalid = message()
        invalid.pose.pose.position.x = math.nan
        self.assertIn("non-finite", self.reason(invalid))

    def test_rejects_invalid_orientation_quaternion(self):
        invalid = message()
        invalid.pose.pose.orientation.w = 0.5
        self.assertIn("quaternion norm", self.reason(invalid))

    def test_real_launch_uses_local_frd_odometry_path_only(self):
        repository = os.path.abspath(
            os.path.join(os.path.dirname(__file__), "..", "..", "..", "..")
        )
        with open(
            os.path.join(repository, "launch", "real.sh"),
            "r",
            encoding="utf-8",
        ) as stream:
            real_launch = stream.read()
        self.assertIn("odom_to_mavros.py", real_launch)
        self.assertIn("/mavros/odometry/out", real_launch)
        self.assertIn("tf tf_echo odom_ned world", real_launch)
        self.assertNotIn("/mavros/vision_pose/pose", real_launch)

        with open(
            os.path.join(
                repository,
                "deployment",
                "ros_pkgs",
                "sim2real_deployment",
                "launch",
                "frame_aliases.launch",
            ),
            "r",
            encoding="utf-8",
        ) as stream:
            aliases = stream.read()
        self.assertIn("world odom", aliases)


if __name__ == "__main__":
    unittest.main()
