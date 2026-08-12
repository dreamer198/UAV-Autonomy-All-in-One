#!/usr/bin/env python3

import importlib.util
import os
import unittest

import numpy as np
import yaml


def load_voxel_map_module():
    path = os.path.join(
        os.path.dirname(__file__),
        "..",
        "scripts",
        "environment_voxel_map.py",
    )
    spec = importlib.util.spec_from_file_location("environment_voxel_map", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class PersistentVoxelMapTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_voxel_map_module()

    def make_map(self, **overrides):
        options = {
            "voxel_size": 0.1,
            "min_range": 0.2,
            "max_range": 44.0,
            "min_z": 0.15,
            "max_z": 3.2,
            "min_hits": 2,
            "max_voxels": 100,
        }
        options.update(overrides)
        return self.module.PersistentVoxelMap(**options)

    def test_rejects_ground_no_return_and_nonfinite_points(self):
        voxel_map = self.make_map(min_hits=1)
        points = np.asarray(
            [
                [1.0, 0.0, 1.0],
                [45.0, 0.0, 1.0],
                [1.0, 0.0, 0.0],
                [np.nan, 0.0, 1.0],
            ],
            dtype=np.float32,
        )
        voxel_map.add(points, np.zeros(3, dtype=np.float32))
        snapshot = voxel_map.points()
        self.assertEqual(snapshot.shape, (1, 3))
        np.testing.assert_allclose(snapshot[0], [1.05, 0.05, 1.05])

    def test_requires_repeated_observation_and_then_persists(self):
        voxel_map = self.make_map()
        wall = np.asarray([[2.01, 1.01, 1.01]], dtype=np.float32)
        voxel_map.add(wall, np.zeros(3, dtype=np.float32))
        self.assertEqual(voxel_map.visible_voxel_count, 0)
        voxel_map.add(wall, np.zeros(3, dtype=np.float32))
        self.assertEqual(voxel_map.visible_voxel_count, 1)

        voxel_map.add(np.empty((0, 3)), np.zeros(3, dtype=np.float32))
        self.assertEqual(voxel_map.visible_voxel_count, 1)
        np.testing.assert_allclose(
            voxel_map.points()[0], [2.05, 1.05, 1.05]
        )

    def test_capacity_drops_only_new_voxels(self):
        voxel_map = self.make_map(min_hits=1, max_voxels=2)
        voxel_map.add(
            np.asarray(
                [[1.0, 0.0, 1.0], [2.0, 0.0, 1.0], [3.0, 0.0, 1.0]],
                dtype=np.float32,
            ),
            np.zeros(3, dtype=np.float32),
        )
        self.assertEqual(voxel_map.voxel_count, 2)
        self.assertEqual(voxel_map.total_dropped_voxels, 1)

    def test_invalid_configuration_is_rejected(self):
        for override in (
            {"voxel_size": 0.0},
            {"max_range": 0.1},
            {"max_z": 0.1},
            {"min_hits": 0},
            {"max_voxels": 0},
        ):
            with self.subTest(override=override):
                with self.assertRaises(ValueError):
                    self.make_map(**override)

    def test_default_rviz_uses_planner_neutral_maps_bounds_and_flight_state(self):
        path = os.path.join(
            os.path.dirname(__file__),
            "..",
            "..",
            "..",
            "config",
            "rviz",
            "sim.rviz",
        )
        with open(path, "r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream)

        manager = config["Visualization Manager"]
        groups = {
            display["Name"]: display
            for display in manager["Displays"]
            if display.get("Class") == "rviz/Group"
        }
        environment = {
            display["Name"]: display
            for display in groups["Environment"]["Displays"]
        }
        self.assertEqual(
            environment["Persistent obstacles"]["Topic"],
            "/planning/viz/environment",
        )
        self.assertIs(environment["Persistent obstacles"]["Enabled"], True)
        self.assertIs(environment["Live lidar"]["Enabled"], False)
        self.assertIs(groups["Planner view"]["Enabled"], True)
        private_map = {
            display["Name"]: display
            for display in groups["Planner view"]["Displays"]
        }
        observed = private_map["Observed obstacles"]
        self.assertIs(observed["Enabled"], True)
        self.assertEqual(float(observed["Alpha"]), 1.0)
        self.assertEqual(observed["Color"], "185; 75; 255")
        self.assertGreater(
            int(observed["Size (Pixels)"]),
            int(environment["Persistent obstacles"]["Size (Pixels)"]),
        )
        self.assertIs(
            private_map["Safety clearance"]["Enabled"], True
        )
        inflated = private_map["Safety clearance"]
        self.assertGreaterEqual(float(inflated["Alpha"]), 0.9)
        self.assertEqual(inflated["Color Transformer"], "AxisColor")
        self.assertEqual(inflated["Axis"], "Z")
        self.assertIs(inflated["Use rainbow"], True)
        self.assertEqual(
            private_map["Fixed planning bounds"]["Marker Topic"],
            "/planning/viz/planning_bounds",
        )
        self.assertIs(
            private_map["Fixed planning bounds"]["Enabled"], True
        )

        planning = {
            display["Name"]: display
            for display in groups["Planning"]["Displays"]
        }
        for name, topic in (
            ("Active planner goal", "/planning/viz/active_goal"),
            ("Actual flight path", "/planning/viz/executed_path"),
        ):
            self.assertIs(planning[name]["Enabled"], True)
            self.assertEqual(planning[name]["Marker Topic"], topic)
        for name in (
            "Backend initial path",
            "Backend trajectory",
        ):
            self.assertIs(planning[name]["Enabled"], True)
        configured_topics = {
            display.get("Topic") or display.get("Marker Topic")
            for group in groups.values()
            for display in group["Displays"]
        }
        self.assertNotIn("/planning/viz/commanded_path", configured_topics)
        self.assertNotIn("/planning/viz/requested_goal", configured_topics)
        self.assertNotIn("/planning/viz/selected_goal", configured_topics)
        self.assertNotIn("/planning/viz/backend/goal", configured_topics)
        self.assertNotIn("/planning/viz/backend/search", configured_topics)
        self.assertEqual(
            manager["Views"]["Current"]["Target Frame"], "world"
        )

    def test_real_rviz_exposes_the_same_human_facing_layers(self):
        path = os.path.join(
            os.path.dirname(__file__),
            "..",
            "..",
            "..",
            "..",
            "deployment",
            "config",
            "rviz",
            "jetson_real_stack.rviz",
        )
        with open(path, "r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream)

        manager = config["Visualization Manager"]
        groups = {
            display["Name"]: display
            for display in manager["Displays"]
            if display.get("Class") == "rviz/Group"
        }
        sensor_layers = {
            display["Name"]: display
            for display in groups["Sensors"]["Displays"]
        }
        self.assertEqual(
            sensor_layers["Persistent obstacles"]["Topic"],
            "/planning/viz/environment",
        )
        self.assertIs(
            sensor_layers["Persistent obstacles"]["Enabled"], True
        )
        self.assertIs(sensor_layers["Registered cloud"]["Enabled"], False)

        vehicle_pose = sensor_layers["UAV pose"]
        self.assertIs(vehicle_pose["Enabled"], True)
        self.assertEqual(vehicle_pose["Topic"], "/localization/odom")
        self.assertEqual(vehicle_pose["Shape"]["Value"], "Axes")

        mapping = {
            display["Name"]: display
            for display in groups["Mapping"]["Displays"]
        }
        observed = mapping["Observed obstacles"]
        self.assertEqual(observed["Color"], "185; 75; 255")
        inflated = mapping["Safety clearance"]
        self.assertIs(inflated["Enabled"], True)
        self.assertGreaterEqual(float(inflated["Alpha"]), 0.9)
        self.assertEqual(inflated["Color Transformer"], "AxisColor")
        self.assertEqual(inflated["Style"], "Points")
        self.assertIs(inflated["Use rainbow"], True)
        panels = config["Panels"]
        self.assertEqual(len(panels), 1)
        self.assertEqual(
            panels[0]["Class"],
            "sim2real_ground_station/InteractiveGoalPanel",
        )
        self.assertEqual(
            manager["Global Options"]["Background Color"], "24; 27; 31"
        )
        self.assertEqual(manager["Global Options"]["Frame Rate"], 30)

        planning = {
            display["Name"]: display
            for display in groups["Planning"]["Displays"]
        }
        for name, topic in (
            ("Active planner goal", "/planning/viz/active_goal"),
            ("Actual flight trajectory", "/planning/viz/executed_path"),
            (
                "Backend initial path",
                "/planning/viz/backend/global_trajectory",
            ),
            ("Backend trajectory", "/planning/viz/backend/trajectory"),
        ):
            self.assertIs(planning[name]["Enabled"], True)
            self.assertEqual(planning[name]["Marker Topic"], topic)

        self.assertEqual(manager["Global Options"]["Fixed Frame"], "world")
        self.assertEqual(
            manager["Tools"][-1]["Topic"],
            "/ground_station/goal_candidate",
        )


if __name__ == "__main__":
    unittest.main()
