#!/usr/bin/env python3

import json
import math
import os
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


class FastPlannerConfigTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        config_dir = os.path.join(os.path.dirname(__file__), "..", "config")
        cls.config_dir = config_dir
        cls.config = {}
        with open(
            os.path.join(config_dir, "planner.yaml"),
            "r",
            encoding="utf-8",
        ) as config_file:
            cls.config["planner"] = yaml.safe_load(config_file)
        with open(
            os.path.join(config_dir, "scenes", "forest.yaml"),
            "r",
            encoding="utf-8",
        ) as config_file:
            cls.config["forest"] = yaml.safe_load(config_file)
        cls.launch_root = ET.parse(
            os.path.join(
                os.path.dirname(__file__), "..", "launch",
                "fast_backend.launch",
            )
        ).getroot()

    def test_every_parameter_has_an_inline_explanation(self):
        paths = (
            os.path.join(self.config_dir, "planner.yaml"),
            os.path.join(self.config_dir, "scenes", "forest.yaml"),
        )
        uncommented = []
        for path in paths:
            with open(path, "r", encoding="utf-8") as config_file:
                lines = config_file.readlines()
            for line_number, line in enumerate(lines, start=1):
                stripped = line.strip()
                if (
                    not stripped
                    or stripped.startswith("#")
                    or stripped.endswith(":")
                ):
                    continue
                if ":" in stripped and "#" not in stripped:
                    uncommented.append(
                        "{}:{}: {}".format(
                            os.path.relpath(path, self.config_dir),
                            line_number,
                            stripped,
                        )
                    )

        self.assertEqual(
            uncommented,
            [],
            "every scalar planner parameter must explain its meaning inline",
        )

    def test_fast_has_one_centered_thirty_meter_horizontal_map(self):
        sdf_map = self.config["planner"]["sdf_map"]
        self.assertEqual(float(sdf_map["resolution"]), 0.1)
        self.assertEqual(float(sdf_map["origin_x"]), -15.0)
        self.assertEqual(float(sdf_map["origin_y"]), -15.0)
        self.assertEqual(float(sdf_map["map_size_x"]), 30.0)
        self.assertEqual(float(sdf_map["map_size_y"]), 30.0)
        self.assertEqual(float(sdf_map["origin_z"]), -1.0)
        self.assertEqual(float(sdf_map["map_size_z"]), 5.0)
        for legacy_name in (
            "base.yaml",
            "local.yaml",
            "forest.yaml",
            "outdoor.yaml",
        ):
            self.assertFalse(
                os.path.exists(os.path.join(self.config_dir, legacy_name))
            )

        voxel_count = math.prod(
            int(math.ceil(float(sdf_map[key]) / float(sdf_map["resolution"])))
            for key in ("map_size_x", "map_size_y", "map_size_z")
        )
        self.assertEqual(voxel_count, 4_500_000)
        self.assertLess(voxel_count * 64, 512 * 1024 * 1024)

    def test_single_core_config_is_loaded_by_planner_and_adapter(self):
        launch_args = {
            element.attrib["name"]: element.attrib
            for element in self.launch_root.findall("arg")
        }
        self.assertEqual(
            launch_args["config"]["default"],
            "$(find sim2real_fast_adapter)/config/planner.yaml",
        )
        self.assertEqual(
            launch_args["forest_scene_config"]["default"],
            "$(find sim2real_fast_adapter)/config/scenes/forest.yaml",
        )
        self.assertNotIn("base_config", launch_args)
        self.assertNotIn("profile_config", launch_args)

        nodes = {
            node.attrib["name"]: node
            for node in self.launch_root.iter("node")
        }
        for node_name in ("planner", "adapter"):
            core_loads = [
                element
                for element in nodes[node_name].findall("rosparam")
                if element.attrib.get("file") == "$(arg config)"
            ]
            self.assertEqual(len(core_loads), 1)

    def test_forest_scene_map_covers_forest_with_local_update_margin(self):
        sdf_map = self.config["forest"]["sdf_map"]
        self.assertEqual(float(sdf_map["resolution"]), 0.2)
        self.assertEqual(float(sdf_map["origin_x"]), -6.0)
        self.assertEqual(float(sdf_map["origin_y"]), -25.0)
        self.assertEqual(float(sdf_map["origin_z"]), -1.0)
        self.assertEqual(float(sdf_map["map_size_x"]), 74.0)
        self.assertEqual(float(sdf_map["map_size_y"]), 31.0)
        self.assertEqual(float(sdf_map["map_size_z"]), 5.0)

        voxel_count = math.prod(
            int(math.ceil(float(sdf_map[key]) / float(sdf_map["resolution"])))
            for key in ("map_size_x", "map_size_y", "map_size_z")
        )
        self.assertEqual(voxel_count, 1_433_750)
        self.assertLess(voxel_count, 4_500_000)
        self.assertLess(voxel_count * 64, 512 * 1024 * 1024)

        repository_root = Path(__file__).resolve().parents[4]
        metadata = json.loads(
            (
                repository_root
                / "simulation/config/scenes/forest/"
                "metadata.json"
            ).read_text(encoding="utf-8")
        )
        minimum = {
            axis: float(sdf_map["origin_{}".format(axis)])
            for axis in ("x", "y", "z")
        }
        maximum = {
            axis: minimum[axis] + float(sdf_map["map_size_{}".format(axis)])
            for axis in ("x", "y", "z")
        }
        inflation = float(
            self.config["planner"]["sdf_map"]["obstacles_inflation"]
        )
        local_map = self.config["planner"]["sdf_map"]
        for index, axis in enumerate(("x", "y")):
            required_margin = (
                float(local_map["local_update_range_{}".format(axis)])
                + inflation
            )
            self.assertLessEqual(
                minimum[axis] + required_margin,
                float(metadata["bounds_min"][index]),
            )
            self.assertGreaterEqual(
                maximum[axis] - required_margin,
                float(metadata["bounds_max"][index]),
            )
        self.assertGreaterEqual(
            maximum["z"],
            float(self.config["planner"]["sdf_map"]["virtual_ceil_height"]),
        )

    def test_forest_scene_override_is_loaded_by_planner_and_adapter(self):
        nodes = {
            node.attrib["name"]: node
            for node in self.launch_root.iter("node")
        }
        expected_condition = (
            "$(eval arg('runtime_mode') == 'simulation' and "
            "arg('scene') == 'forest')"
        )
        for node_name in ("planner", "adapter"):
            overrides = [
                element
                for element in nodes[node_name].findall("rosparam")
                if element.attrib.get("file") == "$(arg forest_scene_config)"
            ]
            self.assertEqual(len(overrides), 1)
            self.assertEqual(overrides[0].attrib.get("if"), expected_condition)

    def test_forest_fast_kino_mission_endpoints_clear_generated_trees(self):
        repository_root = Path(__file__).resolve().parents[4]
        mission = json.loads(
            (repository_root / "mission_forest.json").read_text(encoding="utf-8")
        )
        trees = json.loads(
            (
                repository_root
                / "simulation/config/scenes/forest/trees.json"
            ).read_text(encoding="utf-8")
        )
        base = self.config["planner"]
        goal_clearance = float(base["manager"]["clearance_threshold"])
        mission_endpoint_buffer = 0.5
        required_horizontal_clearance = (
            goal_clearance
            + float(base["sdf_map"]["obstacles_inflation"])
            + mission_endpoint_buffer
        )
        vertical_reach = (
            goal_clearance
            + float(base["sdf_map"]["obstacles_inflation_z"])
        )

        for index, waypoint in enumerate(mission["waypoints"], start=1):
            minimum_surface_distance = min(
                math.hypot(
                    float(waypoint["x"]) - float(tree["x"]),
                    float(waypoint["y"]) - float(tree["y"]),
                )
                - (
                    float(tree["crown_radius"])
                    if float(tree["crown_bottom"])
                    <= float(waypoint["z"]) + vertical_reach
                    else float(tree["trunk_radius"])
                )
                for tree in trees
            )
            self.assertGreater(
                minimum_surface_distance,
                required_horizontal_clearance,
                "forest mission waypoint {} is too close to a generated tree".format(
                    index
                ),
            )

    def test_vertical_update_window_preserves_native_ground_coverage(self):
        sdf_map = self.config["planner"]["sdf_map"]
        adapter = self.config["planner"]["adapter"]
        update_range = float(sdf_map["local_update_range_z"])
        origin = float(sdf_map["origin_z"])

        self.assertEqual(update_range, 4.5)
        self.assertLessEqual(max(origin, 1.5 - update_range), 0.0)
        self.assertIs(adapter["inject_virtual_floor"], True)
        self.assertEqual(float(adapter["virtual_floor_height"]), 0.0)

        base = self.config["planner"]
        resolution = float(sdf_map["resolution"])
        inflated_floor_top = (
            float(adapter["virtual_floor_height"])
            + math.ceil(
                float(base["sdf_map"]["obstacles_inflation_z"]) / resolution
            )
            * resolution
            + 0.5 * resolution
        )
        self.assertLessEqual(
            inflated_floor_top + float(base["topo_prm"]["clearance"]),
            1.0,
        )

    def test_registered_cloud_accumulates_in_static_fast_map(self):
        sdf_map = self.config["planner"]["sdf_map"]
        self.assertIs(sdf_map["accumulate_cloud"], True)
        self.assertEqual(
            int(self.config["planner"]["manager"]["dynamic_environment"]), 0
        )

    def test_public_visualization_is_sourced_before_virtual_floor_injection(self):
        nodes = {
            node.attrib["name"]: node
            for node in self.launch_root.iter("node")
        }
        planner_remaps = {
            remap.attrib["from"]: remap.attrib["to"]
            for remap in nodes["planner"].findall("remap")
        }
        adapter_remaps = {
            remap.attrib["from"]: remap.attrib["to"]
            for remap in nodes["adapter"].findall("remap")
        }

        self.assertEqual(
            planner_remaps["/sdf_map/occupancy_inflate"],
            "/planning/backends/$(arg backend_namespace)/viz/"
            "native_inflated_occupancy",
        )
        self.assertEqual(
            adapter_remaps["viz/occupancy"],
            "/planning/viz/raw/occupancy",
        )
        self.assertEqual(
            adapter_remaps["viz/inflated_occupancy"],
            "/planning/viz/raw/inflated_occupancy",
        )
        self.assertNotIn(
            "/planning/viz/raw/inflated_occupancy",
            planner_remaps.values(),
        )

    def test_topological_segment_retains_native_search_horizon(self):
        manager = self.config["planner"]["manager"]
        self.assertEqual(float(manager["local_segment_length"]), 7.0)

    def test_shared_dynamics_values_are_consistent(self):
        config = self.config["planner"]
        manager = config["manager"]
        search = config["search"]
        optimization = config["optimization"]
        self.assertEqual(float(manager["max_vel"]), float(search["max_vel"]))
        self.assertEqual(
            float(manager["max_vel"]), float(optimization["max_vel"])
        )
        self.assertEqual(float(manager["max_acc"]), float(search["max_acc"]))
        self.assertEqual(
            float(manager["max_acc"]), float(optimization["max_acc"])
        )

    def test_topological_retry_is_rate_limited_below_adapter_timeout(self):
        fsm = self.config["planner"]["fsm"]
        adapter = self.config["planner"]["adapter"]
        retry_interval = float(fsm["planning_retry_interval"])
        planning_timeout = float(adapter["planning_timeout"])

        self.assertTrue(math.isfinite(retry_interval))
        self.assertGreater(retry_interval, 0.0)
        self.assertLess(retry_interval, planning_timeout)
        self.assertEqual(retry_interval, 0.10)

    def test_topological_sampling_covers_staggered_wall_detour(self):
        topo = self.config["planner"]["topo_prm"]
        optimization = self.config["planner"]["optimization"]
        self.assertGreaterEqual(float(topo["sample_inflate_y"]), 5.0)
        self.assertGreaterEqual(int(topo["max_sample_num"]), 4000)
        self.assertGreaterEqual(float(topo["max_sample_time"]), 0.02)
        self.assertGreaterEqual(int(optimization["max_iteration_num1"]), 100)
        self.assertGreaterEqual(float(optimization["max_iteration_time2"]), 0.02)
        self.assertGreaterEqual(
            float(optimization["lambda5"]),
            10.0 * float(optimization["lambda1"]),
        )
        self.assertAlmostEqual(
            float(topo["clearance"])
            - float(self.config["planner"]["manager"]["clearance_threshold"]),
            0.05,
        )
        self.assertGreater(
            float(self.config["planner"]["manager"]["clearance_threshold"])
            + float(self.config["planner"]["sdf_map"]["obstacles_inflation"]),
            math.hypot(0.1663, 0.2368),
        )
        self.assertGreaterEqual(
            float(optimization["dist0"]), float(topo["clearance"])
        )

    def test_measured_arrival_thresholds_are_finite_and_positive(self):
        adapter = self.config["planner"]["adapter"]
        expected = {
            "goal_position_tolerance": 0.35,
            "reached_velocity_tolerance": 0.2,
            "reached_yaw_tolerance_deg": 5.0,
            "reached_yaw_rate_tolerance_deg_s": 10.0,
            "reached_hold_time": 0.5,
        }
        for parameter, expected_value in expected.items():
            value = float(adapter[parameter])
            self.assertTrue(math.isfinite(value))
            self.assertGreater(value, 0.0)
            self.assertEqual(value, expected_value)

    def test_body_self_filter_covers_airframe_without_hiding_free_space(self):
        adapter = self.config["planner"]["adapter"]
        self.assertIs(adapter["self_filter_enabled"], True)
        radius = float(adapter["self_filter_radius"])
        minimum_z = float(adapter["self_filter_min_z"])
        maximum_z = float(adapter["self_filter_max_z"])
        pose_tolerance = float(adapter["self_filter_pose_tolerance"])

        self.assertEqual(radius, 0.35)
        self.assertEqual(minimum_z, -0.20)
        self.assertEqual(maximum_z, 0.20)
        self.assertEqual(pose_tolerance, 0.10)
        self.assertLess(minimum_z, 0.072)
        self.assertGreater(maximum_z, 0.072)
        self.assertGreater(radius, math.hypot(0.1663, 0.2368))
        self.assertLess(radius, 0.4)

    def test_rviz_ground_removal_uses_plane_geometry(self):
        adapter = self.config["planner"]["adapter"]
        self.assertIs(adapter["hide_observed_ground_plane"], True)
        self.assertEqual(
            float(adapter["ground_plane_distance_tolerance"]), 0.03
        )
        self.assertGreaterEqual(
            int(adapter["ground_plane_min_points"]), 200
        )
        self.assertGreaterEqual(
            float(adapter["ground_plane_min_xy_span"]), 2.0
        )


if __name__ == "__main__":
    unittest.main()
