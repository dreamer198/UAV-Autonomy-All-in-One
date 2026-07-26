#!/usr/bin/env python3

import importlib.util
import os
import sys
import types
import unittest
from types import SimpleNamespace
from unittest import mock

def load_real_rviz_bridge():
    rospy = types.ModuleType("rospy")
    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.PoseStamped = object
    mavros_msgs = types.ModuleType("mavros_msgs")
    mavros_msgs_msg = types.ModuleType("mavros_msgs.msg")
    mavros_msgs_msg.State = object
    modules = {
        "rospy": rospy,
        "geometry_msgs": geometry_msgs,
        "geometry_msgs.msg": geometry_msgs_msg,
        "mavros_msgs": mavros_msgs,
        "mavros_msgs.msg": mavros_msgs_msg,
    }
    path = os.path.join(
        os.path.dirname(__file__),
        "..",
        "scripts",
        "rviz_goal_to_diff_planner.py",
    )
    with mock.patch.dict(sys.modules, modules):
        spec = importlib.util.spec_from_file_location("tested_rviz_bridge", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    return module


class RosAdapterSafetyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.rviz_bridge = load_real_rviz_bridge()

    def test_rviz_bridge_never_relabels_a_non_world_goal(self):
        bridge = self.rviz_bridge.RvizGoalToDiffPlanner.__new__(
            self.rviz_bridge.RvizGoalToDiffPlanner
        )
        bridge.frame_id_override = "world"
        bridge.required_frame_id = "world"
        self.assertEqual(
            bridge.resolve_frame(SimpleNamespace(frame_id="map")), ""
        )
        self.assertEqual(
            bridge.resolve_frame(SimpleNamespace(frame_id="")), "world"
        )

    def test_rviz_bridge_uses_inflated_planner_clearance(self):
        bounds = self.rviz_bridge.vertical_clearance_bounds
        minimum, maximum = bounds(0.1, 3.0, 0.33)
        self.assertAlmostEqual(minimum, 0.43)
        self.assertAlmostEqual(maximum, 2.67)

if __name__ == "__main__":
    unittest.main()
