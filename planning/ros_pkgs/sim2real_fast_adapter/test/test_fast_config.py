#!/usr/bin/env python3

import math
import os
import unittest

import yaml


class FastPlannerConfigTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        config_dir = os.path.join(os.path.dirname(__file__), "..", "config")
        cls.config_dir = config_dir
        cls.config = {}
        for name in ("base", "local"):
            with open(
                os.path.join(config_dir, "{}.yaml".format(name)),
                "r",
                encoding="utf-8",
            ) as config_file:
                cls.config[name] = yaml.safe_load(config_file)

    def test_fast_has_one_centered_thirty_meter_horizontal_map(self):
        sdf_map = self.config["local"]["sdf_map"]
        self.assertEqual(float(sdf_map["resolution"]), 0.1)
        self.assertEqual(float(sdf_map["origin_x"]), -15.0)
        self.assertEqual(float(sdf_map["origin_y"]), -15.0)
        self.assertEqual(float(sdf_map["map_size_x"]), 30.0)
        self.assertEqual(float(sdf_map["map_size_y"]), 30.0)
        self.assertEqual(float(sdf_map["origin_z"]), -1.0)
        self.assertEqual(float(sdf_map["map_size_z"]), 5.0)
        self.assertFalse(os.path.exists(os.path.join(self.config_dir, "outdoor.yaml")))

        voxel_count = math.prod(
            int(math.ceil(float(sdf_map[key]) / float(sdf_map["resolution"])))
            for key in ("map_size_x", "map_size_y", "map_size_z")
        )
        self.assertEqual(voxel_count, 4_500_000)
        self.assertLess(voxel_count * 64, 512 * 1024 * 1024)

    def test_vertical_update_window_preserves_native_ground_coverage(self):
        sdf_map = self.config["local"]["sdf_map"]
        adapter = self.config["local"]["adapter"]
        update_range = float(sdf_map["local_update_range_z"])
        origin = float(sdf_map["origin_z"])

        self.assertEqual(update_range, 4.5)
        self.assertLessEqual(max(origin, 1.5 - update_range), 0.0)
        self.assertIs(adapter["inject_virtual_floor"], True)
        self.assertEqual(float(adapter["virtual_floor_height"]), 0.2)

    def test_registered_cloud_accumulates_in_static_fast_map(self):
        sdf_map = self.config["base"]["sdf_map"]
        self.assertIs(sdf_map["accumulate_cloud"], True)
        self.assertEqual(
            int(self.config["base"]["manager"]["dynamic_environment"]), 0
        )

    def test_topological_segment_retains_native_search_horizon(self):
        manager = self.config["base"]["manager"]
        self.assertEqual(float(manager["local_segment_length"]), 7.0)

    def test_topological_sampling_covers_staggered_wall_detour(self):
        topo = self.config["base"]["topo_prm"]
        optimization = self.config["base"]["optimization"]
        self.assertGreaterEqual(float(topo["sample_inflate_y"]), 5.0)
        self.assertGreaterEqual(int(topo["max_sample_num"]), 4000)
        self.assertGreaterEqual(float(topo["max_sample_time"]), 0.02)
        self.assertGreaterEqual(int(optimization["max_iteration_num1"]), 20)
        self.assertGreaterEqual(float(optimization["max_iteration_time2"]), 0.02)
        self.assertGreaterEqual(
            float(topo["clearance"])
            - float(self.config["base"]["manager"]["clearance_threshold"]),
            0.2,
        )
        self.assertGreaterEqual(
            float(optimization["dist0"]), float(topo["clearance"])
        )

    def test_measured_arrival_thresholds_are_finite_and_positive(self):
        adapter = self.config["base"]["adapter"]
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
        adapter = self.config["base"]["adapter"]
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


if __name__ == "__main__":
    unittest.main()
