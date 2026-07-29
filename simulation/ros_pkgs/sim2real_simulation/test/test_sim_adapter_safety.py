#!/usr/bin/env python3

import argparse
import importlib.util
import os
import struct
import sys
import types
import unittest
from types import SimpleNamespace
from unittest import mock

import numpy as np


def load_reconstruction_module():
    rosbag = types.ModuleType("rosbag")
    path = os.path.join(
        os.path.dirname(__file__),
        "..",
        "scripts",
        "reconstruct_bag_world.py",
    )
    with mock.patch.dict(sys.modules, {"rosbag": rosbag}):
        spec = importlib.util.spec_from_file_location(
            "tested_reconstruct_bag_world", path
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    return module


class SimulationAdapterSafetyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reconstruct = load_reconstruction_module()

    def test_numeric_validators_reject_nan_and_infinity(self):
        for validator, value in (
            (self.reconstruct.finite_float, "nan"),
            (self.reconstruct.positive_float, "inf"),
            (self.reconstruct.nonnegative_float, "-inf"),
            (self.reconstruct.unit_interval, "nan"),
        ):
            with self.subTest(validator=validator.__name__, value=value):
                with self.assertRaises(argparse.ArgumentTypeError):
                    validator(value)

    def test_pointcloud_reader_honors_organized_row_padding(self):
        data = bytearray(56)
        points = (
            (1.0, 2.0, 3.0),
            (4.0, 5.0, 6.0),
            (7.0, 8.0, 9.0),
            (10.0, 11.0, 12.0),
        )
        for offset, point in zip((0, 12, 28, 40), points):
            struct.pack_into("<fff", data, offset, *point)
        fields = [
            SimpleNamespace(name=name, offset=offset, datatype=7, count=1)
            for name, offset in (("x", 0), ("y", 4), ("z", 8))
        ]
        cloud = SimpleNamespace(
            fields=fields,
            width=2,
            height=2,
            point_step=12,
            row_step=28,
            is_bigendian=False,
            data=bytes(data),
        )
        np.testing.assert_allclose(
            self.reconstruct.xyz_array(cloud), np.asarray(points)
        )

    def test_pointcloud_reader_supports_float64_fields(self):
        data = struct.pack("<ddd", 1.25, 2.5, 3.75)
        fields = [
            SimpleNamespace(name=name, offset=offset, datatype=8, count=1)
            for name, offset in (("x", 0), ("y", 8), ("z", 16))
        ]
        cloud = SimpleNamespace(
            fields=fields,
            width=1,
            height=1,
            point_step=24,
            row_step=24,
            is_bigendian=False,
            data=data,
        )
        np.testing.assert_allclose(
            self.reconstruct.xyz_array(cloud), [[1.25, 2.5, 3.75]]
        )

    def test_cloud_adapter_never_uses_latest_tf_for_old_cloud(self):
        path = os.path.join(
            os.path.dirname(__file__), "..", "scripts", "pointcloud_to_world.py"
        )
        with open(path, "r", encoding="utf-8") as stream:
            source = stream.read()
        self.assertNotIn("else rospy.Time(0)", source)
        self.assertNotIn(
            "lookup_transform(\n"
            "                    self.target_frame,\n"
            "                    frame_id,\n"
            "                    rospy.Time(0)",
            source,
        )
        self.assertIn(
            "Dropping pointcloud without a measurement timestamp", source
        )
        self.assertIn(
            "Dropping cloud because its measurement-time TF", source
        )

    def test_odometry_relabelling_requires_explicit_identity_contract(self):
        path = os.path.join(
            os.path.dirname(__file__),
            "..",
            "scripts",
            "sim_odometry_adapter.py",
        )
        with open(path, "r", encoding="utf-8") as stream:
            source = stream.read()
        self.assertIn("~allow_identity_frame_alias", source)
        self.assertIn("unexpected frame", source)
        self.assertIn("unexpected child frame", source)

    def test_rviz_world_frame_is_normalized_once(self):
        path = os.path.join(
            os.path.dirname(__file__),
            "..",
            "scripts",
            "rviz_2d_goal_bridge.py",
        )
        with open(path, "r", encoding="utf-8") as stream:
            source = stream.read()
        self.assertIn(').lstrip("/")', source)
        self.assertNotIn("self.frame_id.lstrip", source)

    def test_rviz_bridge_waits_for_nonzero_simulation_time(self):
        path = os.path.join(
            os.path.dirname(__file__),
            "..",
            "scripts",
            "rviz_2d_goal_bridge.py",
        )
        with open(path, "r", encoding="utf-8") as stream:
            source = stream.read()
        self.assertIn("wait_for_nonzero_ros_time(self.state_timeout)", source)
        self.assertIn("time.sleep(min(0.02, remaining))", source)
        self.assertNotIn("rospy.sleep(min(0.02, remaining))", source)


if __name__ == "__main__":
    unittest.main()
